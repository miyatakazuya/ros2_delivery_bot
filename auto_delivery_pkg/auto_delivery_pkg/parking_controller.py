import rclpy
from rclpy.node import Node

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        self.get_logger().info('🅿Parking Logic Node Started (Waiting for Tags...)')

    def control_loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()