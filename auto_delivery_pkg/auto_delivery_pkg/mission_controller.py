import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from enum import Enum, auto
import time

class MissionState(Enum):
    STARTUP = auto()
    WAIT_FOR_START = auto() # NEW: Safety Wait
    SEARCH = auto()       
    ALIGN_FRONT = auto()  
    TURN_PART_1 = auto()  
    TURN_PART_2 = auto() 
    TURN_BUFFER = auto()    # NEW: Pause to let momentum die / camera wake
    BACKUP_PARK = auto()  
    DONE = auto()

# ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}"

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.state = MissionState.STARTUP
        
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.node_control_pub = self.create_publisher(String, 'node_control', 10)

        self.create_subscription(PoseStamped, '/perception/front/tag_pose', self.front_tag_callback, 10)
        self.create_subscription(PoseStamped, '/perception/rear/tag_pose', self.rear_tag_callback, 10)
        
        # Trigger to start mission
        self.create_subscription(Empty, '/mission/start', self.start_callback, 10)

        self.latest_front_dist = None
        self.latest_rear_dist = None
        
        self.boot_time = time.time()
        self.state_start_time = 0.0
        
        # --- TUNING TIMERS ---
        self.WARMUP_DURATION = 3.0
        self.TURN_LEG_DURATION = 1.5 
        self.TURN_OFFSET = 0.4 # Extra time for Leg 2
        self.BUFFER_DURATION = 3.0 # Wait 3s after turn before backing up

        self.timer = self.create_timer(0.1, self.state_loop)
        self.get_logger().info("Mission Controller Initialized.")

    def send_activation(self, node_name: str, value: int):
        msg = String()
        msg.data = f"{node_name}:{value}"
        self.node_control_pub.publish(msg)

    def front_tag_callback(self, msg):
        self.latest_front_dist = msg.pose.position.z

    def rear_tag_callback(self, msg):
        self.latest_rear_dist = msg.pose.position.z

    def start_callback(self, msg):
        if self.state == MissionState.WAIT_FOR_START:
            self.get_logger().info("Start Signal Received! -> SEARCH")
            self.state = MissionState.SEARCH

    def state_loop(self):
        current_time = time.time()
        
        # --- STATE MACHINE ---
        
        if self.state == MissionState.STARTUP:
            if (current_time - self.boot_time) > self.WARMUP_DURATION:
                self.get_logger().info("Warmup Done. Waiting for Start Signal...")
                # Pre-warm vision nodes? Or wait? 
                # Let's turn them on so they are ready
                self.send_activation("oak_perception_node", 1) 
                self.send_activation("apriltag_node", 1)
                self.send_activation("webcam_apriltag", 0)
                self.state = MissionState.WAIT_FOR_START

        elif self.state == MissionState.WAIT_FOR_START:
            # Do nothing until /mission/start received
            self.state = MissionState.SEARCH
            pass

        elif self.state == MissionState.SEARCH:
            if self.latest_front_dist and self.latest_front_dist < 1.0:
                self.get_logger().info("Target Locked -> ALIGN_FRONT")
                self.state = MissionState.ALIGN_FRONT

        elif self.state == MissionState.ALIGN_FRONT:
            if self.latest_front_dist and self.latest_front_dist < 0.6:
                self.get_logger().info("Aligned -> STARTING 180 TURN (Leg 1)")
                self.state = MissionState.TURN_PART_1
                self.state_start_time = time.time()

        elif self.state == MissionState.TURN_PART_1:
            if (current_time - self.state_start_time) > self.TURN_LEG_DURATION:
                self.get_logger().info("Leg 1 Done -> STARTING Leg 2")
                self.state = MissionState.TURN_PART_2
                self.state_start_time = time.time() 

        elif self.state == MissionState.TURN_PART_2:
            # Added Offset for momentum
            if (current_time - self.state_start_time) > (self.TURN_LEG_DURATION + self.TURN_OFFSET):
                self.get_logger().info("Turn Complete -> BUFFER (Wake Rear Cam)")
                self.state = MissionState.TURN_BUFFER
                self.state_start_time = time.time()
                
                self.send_activation("webcam_apriltag", 1) 
                # Optional: Turn off front cam
                self.send_activation("oak_perception_node", 0)

        elif self.state == MissionState.TURN_BUFFER:
            # Just wait for camera to stabilize and momentum to die
            if (current_time - self.state_start_time) > self.BUFFER_DURATION:
                self.get_logger().info("Buffer Done -> BACKUP_PARK")
                self.state = MissionState.BACKUP_PARK

        elif self.state == MissionState.BACKUP_PARK:
            if self.latest_rear_dist and self.latest_rear_dist < 0.2:
                self.get_logger().info("Docked -> DONE")
                self.state = MissionState.DONE

        # Publish State
        msg = String()
        msg.data = self.state.name 
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()