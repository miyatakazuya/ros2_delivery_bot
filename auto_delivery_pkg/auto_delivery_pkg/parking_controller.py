import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')

        # Start inactive
        self.active = False  

        # Subscribe to node control signals
        self.command_sub = self.create_subscription(
            String,
            'node_control',
            self.command_callback,
            10
        )

        # Timer for control loop (5 Hz)
        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info('🅿 ParkingController initialized — waiting for activation signal')

    def command_callback(self, msg: String):
        """
        Expected format: "parking_controller:1" or "parking_controller:0"
        """
        command = msg.data.strip()

        try:
            target, state = command.split(':')
        except ValueError:
            self.get_logger().warn(f"Malformed command: '{command}'")
            return

        # Ignore unrelated commands
        if target != 'parking_controller':
            return

        if state not in ('0', '1'):
            self.get_logger().warn(f"Unknown state '{state}' for parking_controller")
            return

        new_state = (state == '1')

        if new_state != self.active:
            self.active = new_state
            self.get_logger().info(
                f"Activation → {'ACTIVE' if self.active else 'IDLE'}"
            )

    def control_loop(self):
        # Node stays idle unless active
        if not self.active:
            return

        # Here goes your real parking logic later
        self.get_logger().info("Parking control loop running...")

def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()