import rclpy
from rclpy.node import Node
from enum import IntEnum, auto

class MissionState(IntEnum):
    IDLE = auto()
    SEARCHING = auto()
    PARKING = auto()
    DELIVERING = auto()
    RETURNING = auto()

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
                self.state = MissionState.IDLE
        
        # Control loop timer (I made it 1 sec at random so change if need be)
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info('Mission Controller Node Started')

    def control_loop(self):
        self.get_logger().info(f'CURRENT STATE: [{self.state.name}]')

        if self.state == MissionState.IDLE:
            self.get_logger().info("Waiting for mission start...")
            self.state = MissionState.SEARCHING

        elif self.state == MissionState.SEARCHING:
            self.get_logger().info("Locating apriltagS")
            self.state = MissionState.PARKING

        elif self.state == MissionState.PARKING:
            self.get_logger().info("Backing up")
            self.state = MissionState.DELIVERING

        elif self.state == MissionState.DELIVERING:
            self.get_logger().info("Dropping Package")
            self.state = MissionState.RETURNING

        elif self.state == MissionState.RETURNING:
            self.get_logger().info("Mission Complete.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()