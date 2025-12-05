import time
import math
import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import String
from threading import Thread

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
        newmode = (oldmode & 0x7F) | 0x10  # Set SLEEP bit (bit 4)
        
        self.write_byte(0x00, newmode)     # Go to sleep (required to set prescale)
        self.write_byte(0xFE, prescale)    # Write prescale value
        
        # --- THE FIX IS HERE ---
        # We must clear the SLEEP bit (bit 4) to wake it up
        # We also usually want Auto-Increment (bit 5) enabled for multiple byte writes
        self.write_byte(0x00, oldmode & 0xEF) # 0xEF = 11101111 (Clears bit 4)

        time.sleep(0.005) # Wait for oscillator to stabilize

        # Enable Restart (bit 7) and keep Sleep cleared
        self.write_byte(0x00, (oldmode & 0xEF) | 0x80)

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
        
        # --- Parameters ---
        # Allow these to be configured via launch files or command line
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('servo_channel', 0)
        self.declare_parameter('close_loc', 1950)
        self.declare_parameter('open_loc', 1500)
        self.declare_parameter('test', False)

        # Retrieve parameter values
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.servo_channel = self.get_parameter('servo_channel').value
        self.open_loc = self.get_parameter('open_loc').value
        self.close_loc = self.get_parameter('close_loc').value
        self.test_mode = self.get_parameter('test').value

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

        # --- ROS Interfaces ---
        self.subscription = self.create_subscription(
            String,
            '/servo',
            self.command_callback,
            10,
        )

        if self.test_mode:
            Thread(target=self.run_startup_test, daemon=True).start()

    def run_startup_test(self):
        if self.pca is None:
            self.get_logger().warn('Hardware not connected, skipping startup test sequence.')
            return

        self.get_logger().info('Running startup servo test sequence')
        self._move_servo(self.open_loc, 'OPEN (test)')
        time.sleep(1)
        self._move_servo(self.close_loc, 'CLOSE (test)')
        time.sleep(1)
        self._move_servo(self.close_loc, 'CLOSE (final)')

    def command_callback(self, msg: String):
        if self.pca is None:
            self.get_logger().warn('Hardware not connected, cannot move servo.')
            return

        command = msg.data.strip().lower()
        if command == 'open':
            self._move_servo(self.open_loc, 'OPEN')
        elif command == 'close':
            self._move_servo(self.close_loc, 'CLOSE')
        else:
            self.get_logger().warn(f"Received unknown servo command: '{msg.data}' (expected 'open' or 'close')")

    def _move_servo(self, target_pulse: int, status_text: str):
        try:
            self.pca.set_servo_pulse(self.servo_channel, target_pulse)
            self.get_logger().info(f'Servo Controller: Tray is {status_text} ({target_pulse}us)')
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