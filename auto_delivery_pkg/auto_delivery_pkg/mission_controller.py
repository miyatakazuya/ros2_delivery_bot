import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from enum import Enum, auto
import time
import json  

class MissionState(Enum):
    STARTUP = auto()
    WAIT_FOR_START = auto()
    SEARCH = auto()       
    ALIGN_FRONT = auto()  
    TURN_PART_1 = auto()  
    TURN_PART_2 = auto() 
    TURN_BUFFER = auto()
    BACKUP_PARK = auto()  
    BLIND_DOCK = auto()   
    DONE = auto()

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.state = MissionState.STARTUP
        
        # --- Publishers ---
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.node_control_pub = self.create_publisher(String, 'node_control', 10)
        self.servo_pub = self.create_publisher(String, '/servo', 10)

        # --- Subscribers ---
        self.create_subscription(PoseStamped, '/perception/front/tag_pose', self.front_tag_callback, 10)
        self.create_subscription(PoseStamped, '/perception/rear/tag_pose', self.rear_tag_callback, 10)
        self.create_subscription(Empty, '/mission/start', self.start_callback, 10)
        self.create_subscription(String, '/perception/front/yolo_data', self.yolo_callback, 10)

        self.latest_front_dist = None
        self.latest_rear_dist = None

        self.container_filled = False
        
        self.boot_time = time.time()
        self.state_start_time = 0.0
        
        # --- TUNING TIMERS ---
        self.WARMUP_DURATION = 3.0
        self.TURN_LEG_DURATION = 1.5 
        self.TURN_OFFSET = 0.4 
        self.BUFFER_DURATION = 3.0 
        
        self.HANDOVER_DIST = 0.35 
        self.BLIND_DURATION = 0.1 

        self.timer = self.create_timer(0.1, self.state_loop)
        self.get_logger().info("Mission Controller Initialized.")

    def send_activation(self, node_name: str, value: int):
        msg = String()
        msg.data = f"{node_name}:{value}"
        self.node_control_pub.publish(msg)

    def trigger_servo(self, command: str):
        msg = String()
        msg.data = command
        self.servo_pub.publish(msg)
        self.get_logger().info(f"Servo Command Sent: {command}")

    def front_tag_callback(self, msg):
        self.latest_front_dist = msg.pose.position.z

    def rear_tag_callback(self, msg):
        self.latest_rear_dist = msg.pose.position.z

    def start_callback(self, msg):
        if self.state == MissionState.WAIT_FOR_START:
            self.get_logger().info("Start Signal Received! -> SEARCH")
            self.state = MissionState.SEARCH
            
    def yolo_callback(self, msg: String):
        """
        Called when YOLO detections arrive from oak_perception_node.
        If a 'filled container' is seen, mark mission as done.
        """
        if self.container_filled:
            # Already decided, no need to re-check
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to parse YOLO JSON")
            return

        FILLED_LABELS = {"Red Box", "red box"}

        for det in detections:
            label = det.get("label", "")
            if label in FILLED_LABELS:
                self.container_filled = True
                self.get_logger().info("Container is filled – stopping mission.")
                break

    def state_loop(self):
        current_time = time.time()

        if self.container_filled and self.state != MissionState.DONE:
            self.get_logger().info("Container already filled. Aborting mission and setting state to DONE.")
            self.state = MissionState.DONE

        if self.state == MissionState.STARTUP:
            self.trigger_servo("close")
            self.send_activation("oak_perception_node", 1) 
            if (current_time - self.boot_time) > self.WARMUP_DURATION:
                self.get_logger().info("Warmup Done. Waiting for Start Signal...")
                self.send_activation("apriltag_node", 1)
                self.send_activation("webcam_apriltag", 0)
                self.state = MissionState.WAIT_FOR_START

        elif self.state == MissionState.WAIT_FOR_START:
            # Uncomment this if u don't want the mission to halt on startup
            # self.state = MissionState.SEARCH
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
            if (current_time - self.state_start_time) > (self.TURN_LEG_DURATION + self.TURN_OFFSET):
                self.get_logger().info("Turn Complete -> BUFFER (Wake Rear Cam)")
                self.state = MissionState.TURN_BUFFER
                self.state_start_time = time.time()
                self.send_activation("webcam_apriltag", 1) 
                self.send_activation("oak_perception", 0)

        elif self.state == MissionState.TURN_BUFFER:
            if (current_time - self.state_start_time) > self.BUFFER_DURATION:
                self.get_logger().info("Buffer Done -> BACKUP_PARK")
                self.state = MissionState.BACKUP_PARK

        elif self.state == MissionState.BACKUP_PARK:
            self.get_logger().info(f"rear_dist ({self.latest_rear_dist}m)")

            if self.latest_rear_dist and self.latest_rear_dist < self.HANDOVER_DIST:
                self.get_logger().info(f"Close ({self.latest_rear_dist:.2f}m) -> BLIND DOCKING")
                self.state = MissionState.BLIND_DOCK
                self.state_start_time = time.time()

        elif self.state == MissionState.BLIND_DOCK:
            if (current_time - self.state_start_time) > self.BLIND_DURATION:
                self.get_logger().info("Blind Dock Complete -> DROPPING")
                self.trigger_servo("open")
                self.state = MissionState.DONE

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