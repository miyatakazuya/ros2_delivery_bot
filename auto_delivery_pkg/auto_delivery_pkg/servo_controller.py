import time
import math
import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import String

# ==========================================
# PCA9685 Driver Class
# ==========================================
class PCA9685:
    """
    Minimal driver for the PCA9685 PWM controller using smbus2.
    """
    def __init__(self, channel=1, address=0x40, frequency=50):
        self.address = address
        try:
            self.bus = SMBus(channel)
        except FileNotFoundError:
            # Re-raising the error so the ROS node can handle logging it
            raise FileNotFoundError(f"Error: /dev/i2c-{channel} not found.")

        self.set_pwm_freq(frequency)

    def write_byte(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq_hz):
        """Sets the PWM frequency in Hz."""
        prescale_val = 25000000.0  # 25MHz internal oscillator
        prescale_val /= 4096.0     # 12-bit
        prescale_val /= float(freq_hz)
        prescale_val -= 1.0
        prescale = int(math.floor(prescale_val + 0.5))

        oldmode = self.read_byte(0x00)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write_byte(0x00, newmode)     # go to sleep
        self.write_byte(0xFE, prescale)
        self.write_byte(0x00, oldmode)
        time.sleep(0.005)
        self.write_byte(0x00, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        """Sets the start (on) and end (off) tick for a specific channel."""
        reg_base = 0x06 + 4 * channel
        self.bus.write_byte_data(self.address, reg_base, on & 0xFF)
        self.bus.write_byte_data(self.address, reg_base + 1, on >> 8)
        self.bus.write_byte_data(self.address, reg_base + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, reg_base + 3, off >> 8)

    def set_servo_pulse(self, channel, pulse_us):
        """Helper to set pulse in microseconds (assuming 50Hz)."""
        period_us = 20000.0  # 50Hz = 20ms = 20,000us
        # 4096 ticks per cycle
        pulse_length = 4096.0 * (pulse_us / period_us)
        self.set_pwm(channel, 0, int(pulse_length))


# ==========================================
# ROS2 Node
# ==========================================
class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # NEW: acrivation flag + subscriber to /node_control
        self.active = False
        self.command_sub = self.create_subscription(
            String,
            'node_control',
            self.command_callback,
            10
        )
        
        # --- Parameters ---
        # Allow these to be configured via launch files or command line
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('servo_channel', 0)
        
        # Retrieve parameter values
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.servo_channel = self.get_parameter('servo_channel').value

        self.get_logger().info(f'Initializing PCA9685 on Bus {self.i2c_bus}, Addr {hex(self.i2c_address)}, Servo Ch {self.servo_channel}')

        # --- Hardware Initialization ---
        self.pca = None
        try:
            self.pca = PCA9685(channel=self.i2c_bus, address=self.i2c_address)
            self.get_logger().info('Hardware Connected Successfully')
        except FileNotFoundError:
            self.get_logger().fatal(f'Could not open /dev/i2c-{self.i2c_bus}. Ensure I2C is enabled and docker args are correct.')
            # We don't exit here to keep the node alive for debugging, 
            # but functionality will be disabled.
        except Exception as e:
            self.get_logger().error(f'Unexpected error connecting to PCA9685: {e}')

        # --- Logic ---
        # Timer to toggle servo position every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.is_open = False # State tracker
        
    # NEW: handle "servo_controller:1"/"servo_controller:0"
    def command_callback(self, msg: String):
        command = msg.data.strip()
        try:
            target, state = command.split(':')
        except ValueError:
            self.get_logger().warn(f"Malformed command: '{command}'")
            return

        if target != 'servo_controller':
            return

        if state not in ('0', '1'):
            self.get_logger().warn(f"Unknown state '{state}' for servo_controller")
            return

        new_active = (state == '1')
        if new_active != self.active:
            self.active = new_active
            self.get_logger().info(
                f"Activation → {'ACTIVE' if self.active else 'IDLE'}"
            )


    def timer_callback(self):
        # NEW: only run this loop when ACTIVE
        if not self.active:
            return
        
        if self.pca is None:
            self.get_logger().warn('Hardware not connected, skipping control loop.')
            return

        try:
            if self.is_open:
                # Move to Closed Position (e.g., 1000us)
                target_pulse = 1000
                status_text = "CLOSED"
            else:
                # Move to Open Position (e.g., 2000us)
                target_pulse = 2000
                status_text = "OPEN"

            self.pca.set_servo_pulse(self.servo_channel, target_pulse)
            
            self.get_logger().info(f'Servo Controller: Tray is {status_text} ({target_pulse}us)')
            
            # Toggle state for next run
            self.is_open = not self.is_open

        except Exception as e:
            self.get_logger().error(f'Error writing to I2C bus: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Optional: Reset servo to "Closed" or "Center" on shutdown
        if node.pca:
            node.get_logger().info('Shutting down, resetting servo to 1500us...')
            try:
                node.pca.set_servo_pulse(node.servo_channel, 1500)
                # Give it a moment to move before killing the script
                time.sleep(0.5) 
            except:
                pass
                
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()