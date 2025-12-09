import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from enum import Enum, auto
import time

class MissionState(Enum):
    STARTUP = auto()
    SEARCH = auto()       
    ALIGN_FRONT = auto()  
    TURN_AROUND = auto()  
    BACKUP_PARK = auto()  
    DONE = auto()

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.state = MissionState.STARTUP
        
        # --- Publishers ---
        # 1. Instructs Motion Controller what to do
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        # 2. Toggles Vision Nodes
        self.node_control_pub = self.create_publisher(String, 'node_control', 10)

        # --- Subscribers ---
        # We still need pose to make Decisions (Transition logic)
        self.create_subscription(PoseStamped, '/perception/front/tag_pose', self.front_tag_callback, 10)
        self.create_subscription(PoseStamped, '/perception/rear/tag_pose', self.rear_tag_callback, 10)

        # --- Variables ---
        self.latest_front_dist = None
        self.latest_rear_dist = None
        
        # Timers
        self.boot_time = time.time()
        self.turn_start_time = 0.0
        
        # Constants
        self.WARMUP_DURATION = 5.0
        self.TURN_DURATION = 3.5 

        # Low Frequency Loop (10Hz is plenty for state machine)
        self.timer = self.create_timer(0.1, self.state_loop)
        self.get_logger().info("Mission Controller (Brain) Initialized.")

    def send_activation(self, node_name: str, value: int):
        msg = String()
        msg.data = f"{node_name}:{value}"
        self.node_control_pub.publish(msg)

    def front_tag_callback(self, msg):
        self.latest_front_dist = msg.pose.position.z

    def rear_tag_callback(self, msg):
        self.latest_rear_dist = msg.pose.position.z

    def state_loop(self):
        current_time = time.time()
        
        # --- STATE MACHINE TRANSITIONS ---
        
        if self.state == MissionState.STARTUP:
            elapsed = current_time - self.boot_time
            if elapsed > self.WARMUP_DURATION:
                self.get_logger().info("Warmup Done -> SEARCH")
                self.send_activation("oak_perception_node", 1) 
                self.send_activation("apriltag_node", 1)
                self.send_activation("webcam_apriltag", 0)
                self.state = MissionState.SEARCH

        elif self.state == MissionState.SEARCH:
            # Transition: If we see tag and are close enough
            if self.latest_front_dist and self.latest_front_dist < 1.0:
                self.get_logger().info("Target Locked -> ALIGN_FRONT")
                self.state = MissionState.ALIGN_FRONT

        elif self.state == MissionState.ALIGN_FRONT:
            # Transition: Stopping distance reached
            if self.latest_front_dist and self.latest_front_dist < 0.6:
                self.get_logger().info("Aligned -> TURN_AROUND")
                self.state = MissionState.TURN_AROUND
                self.turn_start_time = time.time()
                self.send_activation("webcam_apriltag", 1) 

        elif self.state == MissionState.TURN_AROUND:
            # Transition: Timer based (IMU upgrade goes here later)
            if (current_time - self.turn_start_time) > self.TURN_DURATION:
                self.get_logger().info("Turn Complete -> BACKUP_PARK")
                self.state = MissionState.BACKUP_PARK

        elif self.state == MissionState.BACKUP_PARK:
            # Transition: Docked
            if self.latest_rear_dist and self.latest_rear_dist < 0.2:
                self.get_logger().info("Docked -> DONE")
                self.state = MissionState.DONE

        # --- PUBLISH STATE ---
        # The Motion Controller listens to this string
        msg = String()
        msg.data = self.state.name # e.g. "SEARCH", "ALIGN_FRONT"
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()