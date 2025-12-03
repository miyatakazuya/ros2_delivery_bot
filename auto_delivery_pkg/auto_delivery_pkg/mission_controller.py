import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum, auto

class MissionState(Enum):
    IDLE = auto()
    SEARCHING = auto()
    PARKING = auto()
    RETURNING = auto()

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        self.state = MissionState.IDLE

        self.node_control_pub = self.create_publisher(String, 'node_control', 10)

        # State machine timer (every 3 seconds)
        self.timer = self.create_timer(3.0, self.state_step)

        self.get_logger().info("MissionController initialized")

    def send_activation(self, node_name: str, value: int):
        """
        value: 1 = activate, 0 = deactivate
        """
        msg = String()
        msg.data = f"{node_name}:{value}"
        self.node_control_pub.publish(msg)
        self.get_logger().info(f"Sent → {msg.data}")

    def state_step(self):
        if self.state == MissionState.IDLE:
            self.get_logger().info("IDLE → SEARCHING")
            self.state = MissionState.SEARCHING

        elif self.state == MissionState.SEARCHING:
            self.get_logger().info("SEARCHING → PARKING (simulate detected container)")
            self.send_activation("parking_controller", 1)   # Turn ON
            self.state = MissionState.PARKING

        elif self.state == MissionState.PARKING:
            self.get_logger().info("PARKING → RETURNING (simulate finished parking)")
            self.send_activation("parking_controller", 0)   # Turn OFF
            self.state = MissionState.RETURNING

        elif self.state == MissionState.RETURNING:
            self.get_logger().info("RETURNING: mission complete.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()