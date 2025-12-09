import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from enum import Enum, auto
import json
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
        self.node_control_pub = self.create_publisher(String, 'node_control', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/perception/front/yolo_data', self.yolo_callback, 10)
        self.create_subscription(PoseStamped, '/perception/front/tag_pose', self.front_tag_callback, 10)
        self.create_subscription(PoseStamped, '/perception/rear/tag_pose', self.rear_tag_callback, 10)

        # --- State Variables ---
        self.latest_front_tag = None
        self.latest_rear_tag = None
        self.last_tag_time = 0.0
        
        # Turn Timer
        self.turn_start_time = 0.0
        self.TURN_DURATION = 3.5 # TODO: still need to tune ts manually

        self.boot_time = time.time()
        self.WARMUP_DURATION = 5.0 # Wait 5 seconds for OAK-D to stabilize

        self.timer = self.create_timer(0.1, self.control_loop) # 10hz i think
        self.get_logger().info("Mission Controller Initialized (Ackermann Logic).")

    def send_activation(self, node_name: str, value: int):
        msg = String()
        msg.data = f"{node_name}:{value}"
        self.node_control_pub.publish(msg)

    def yolo_callback(self, msg):
        # Yolo is ran here but we dont do anything with the results yet
        # self.get_logger().info(f"YOLO Seen: {msg.data}")
        pass

    def front_tag_callback(self, msg):
        self.latest_front_tag = msg.pose.position
        self.last_tag_time = time.time()

    def rear_tag_callback(self, msg):
        self.latest_rear_tag = msg.pose.position

    # Main Control Loop
    def control_loop(self):
        cmd = Twist()
        current_time = time.time()
        
        # Check if Front Tag is to avoid working off stale data
        tag_is_visible = (self.latest_front_tag is not None) and \
                         (current_time - self.last_tag_time < 0.5)

        # =========================================================
        # STATE 1: STARTUP
        # =========================================================
        if self.state == MissionState.STARTUP:
            elapsed = current_time - self.boot_time
            if elapsed < self.WARMUP_DURATION:
                self.get_logger().info(f"Warming up sensors... {int(self.WARMUP_DURATION - elapsed)}s", throttle_duration_sec=1.0)
                self.send_activation("oak_perception_node", 1) 
                self.send_activation("apriltag_node", 1)
                self.send_activation("webcam_apriltag", 0)
                return # EXIT LOOP (Don't move yet)

            # self.send_activation("oak_perception_node", 1) 
            # self.send_activation("apriltag_node", 1)
            # self.send_activation("webcam_apriltag", 0)
            
            self.get_logger().info("Startup Complete -> Switching to SEARCH")
            self.state = MissionState.SEARCH

        # =========================================================
        # STATE 2: SEARCH (Scan & Approach)
        # =========================================================
        elif self.state == MissionState.SEARCH:
            
            if not tag_is_visible:
                # TODO: this is in the case that the tag is not in the view of the car on startup.
                # Not sure if turning in a circle until finding it makes total sense but will test (i think)
                cmd.linear.x = 0.0  
                cmd.angular.z = 0.0  
            
            else:               
                error_x = self.latest_front_tag.x 
                
                k_p = 0.5 # TODO: tune this
                cmd.angular.z = k_p * error_x 
                cmd.linear.x = 0.2 
                
                self.get_logger().info(f"Error: {error_x:.2f}")
                self.get_logger().info(f"Tracking Tag | Dist: {self.latest_front_tag.z:.2f}m")


                if self.latest_front_tag.z < 1.0: # 1 meter away
                    self.get_logger().info("Close Range -> Switching to ALIGN_FRONT")
                    self.state = MissionState.ALIGN_FRONT

        # =========================================================
        # STATE 3: ALIGN_FRONT (Precision)
        # =========================================================
        elif self.state == MissionState.ALIGN_FRONT:
            # Again case if tag is somehow not visible.
            if not tag_is_visible:
                cmd.linear.x = 0.0
                self.get_logger().warn("Tag Lost")
            else:
                error_x = self.latest_front_tag.x
                cmd.linear.x = 0.15
                cmd.angular.z = 0.3 * error_x 
                
                # Stop Logic 
                if self.latest_front_tag.z < 0.6: # 0.6m 
                    cmd.linear.x = 0.0
                    self.get_logger().info("180 time")
                    
                    self.state = MissionState.TURN_AROUND
                    self.turn_start_time = time.time()
                    
                    # Enable rear cam
                    self.send_activation("webcam_apriltag", 1) 
                    # self.send_activation("oak_perception_node", 0) 

        # =========================================================
        # STATE 4: TURN_AROUND (Blind 180)
        # =========================================================
        elif self.state == MissionState.TURN_AROUND:
            elapsed = time.time() - self.turn_start_time
            
            if elapsed < self.TURN_DURATION:
                cmd.linear.x = 0.5  
                cmd.angular.z = 1.0 
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("Turn Complete. ")
#                self.state = MissionState.BACKUP_PARK

        # =========================================================
        # STATE 5: BACKUP_PARK (Reverse Docking)
        # =========================================================
        elif self.state == MissionState.BACKUP_PARK:
            
            if self.latest_rear_tag:  
                dist = self.latest_rear_tag.z # Distance behind robot
                error_x = self.latest_rear_tag.x
                
                if dist < 0.2: # idk if the car can see anything after this
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info("DOCKED!")   
                    self.state = MissionState.DONE

                    #TODO: Do servo stuff here
                else:
                    cmd.linear.x = -0.15 # Backing up speed 
                    k_p_rear = 2.0
                    cmd.angular.z = -k_p_rear * error_x 
            
            else:
                # Tag not seen (skill issue)
                cmd.linear.x = 0.0
                self.get_logger().info("Searching for Rear Tag...")

        # =========================================================
        # FINAL: PUBLISH
        # =========================================================
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
