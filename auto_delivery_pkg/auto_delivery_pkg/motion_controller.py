import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import time

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(String, '/mission/state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/perception/front/tag_pose', self.front_tag_callback, 10)
        self.create_subscription(PoseStamped, '/perception/rear/tag_pose', self.rear_tag_callback, 10)

        self.current_state = "STARTUP"
        self.front_tag = None
        self.rear_tag = None
        self.last_front_time = 0.0
        self.last_rear_time = 0.0
        
        # Tuning Gains
        self.KP_SEARCH = 0.3
        self.KP_ALIGN = 0.3
        self.KP_REVERSE = 2.0

        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz
        self.get_logger().info("Motion Controller Initialized.")

    def state_callback(self, msg):
        self.current_state = msg.data

    def front_tag_callback(self, msg):
        self.front_tag = msg.pose.position
        self.last_front_time = time.time()

    def rear_tag_callback(self, msg):
        self.rear_tag = msg.pose.position
        self.last_rear_time = time.time()

    def control_loop(self):
        cmd = Twist()
        current_time = time.time()
        
        # --- DATA FRESHNESS SETTINGS ---
        # Front OAK-D is fast (30Hz+), so 0.5s is generous but safe.
        front_timeout = 0.5 
        # Rear USB Webcam is SLOW/Laggy. 
        # We allow "holding" the command for up to 1.0 second to bridge the gap between frames.
        rear_timeout = 1.0 

        front_fresh = (current_time - self.last_front_time) < front_timeout
        rear_fresh = (current_time - self.last_rear_time) < rear_timeout

        # --- STATE: STARTUP / WAIT / DONE ---
        if self.current_state in ["STARTUP", "WAIT_FOR_START", "TURN_BUFFER", "DONE"]:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # --- STATE: SEARCH ---
        elif self.current_state == "SEARCH":
            if not front_fresh:
                # If truly lost for >0.5s, stop to be safe
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                err_x = self.front_tag.x
                cmd.linear.x = 0.2
                cmd.angular.z = self.KP_SEARCH * err_x 

        # --- STATE: ALIGN_FRONT ---
        elif self.current_state == "ALIGN_FRONT":
            if front_fresh:
                err_x = self.front_tag.x
                cmd.linear.x = 0.15 
                cmd.angular.z = self.KP_ALIGN * err_x
            else:
                cmd.linear.x = 0.0

        # --- STATE: TURNS ---
        elif self.current_state == "TURN_PART_1":
            cmd.linear.x = -0.3 
            cmd.angular.z = -1.0 

        elif self.current_state == "TURN_PART_2":
            cmd.linear.x = 0.3
            cmd.angular.z = 1.0 

        # --- STATE: BACKUP_PARK (The Fix) ---
        elif self.current_state == "BACKUP_PARK":
            # We use the permissive 'rear_fresh' (1.0s) check here
            if self.rear_tag and rear_fresh:
                err_x = self.rear_tag.x
                cmd.linear.x = -0.15
                cmd.angular.z = -self.KP_REVERSE * err_x 
            else:
                # Only STOP if data is OLDER than 1.0s
                # This bridges the gaps between slow webcam frames
                if self.rear_tag: 
                    self.get_logger().warn(f"Rear Cam Lagging! Last seen: {current_time - self.last_rear_time:.2f}s ago")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()