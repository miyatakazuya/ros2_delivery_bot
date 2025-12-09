import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import time

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        
        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/mission/state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/perception/front/tag_pose', self.front_tag_callback, 10)
        self.create_subscription(PoseStamped, '/perception/rear/tag_pose', self.rear_tag_callback, 10)

        # --- Variables ---
        self.current_state = "STARTUP"
        self.front_tag = None
        self.rear_tag = None
        self.last_tag_time = 0.0

        # High Frequency Control Loop (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Motion Controller (Reflexes) Initialized @ 50Hz")

    def state_callback(self, msg):
        self.current_state = msg.data

    def front_tag_callback(self, msg):
        self.front_tag = msg.pose.position
        self.last_tag_time = time.time()

    def rear_tag_callback(self, msg):
        self.rear_tag = msg.pose.position

    def control_loop(self):
        cmd = Twist()
        
        # Safety: Check if tag data is stale (>0.5s old)
        tag_is_fresh = (time.time() - self.last_tag_time) < 0.5

        # --- STATE: STARTUP / DONE ---
        if self.current_state in ["STARTUP", "DONE"]:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # --- STATE: SEARCH ---
        elif self.current_state == "SEARCH":
            if not tag_is_fresh:
                # Scan Pattern
                cmd.linear.x = 0.15
                cmd.angular.z = 0.6
            else:
                # Visual Servo (Fast Approach)
                err_x = self.front_tag.x
                cmd.linear.x = 0.2
                # P-Control: 50Hz update means we can be responsive
                cmd.angular.z = 1.0 * err_x 

        # --- STATE: ALIGN_FRONT ---
        elif self.current_state == "ALIGN_FRONT":
            if tag_is_fresh:
                err_x = self.front_tag.x
                # Slower speed, tighter steering
                cmd.linear.x = 0.15 
                cmd.angular.z = 1.5 * err_x
            else:
                # Safety Stop
                cmd.linear.x = 0.0

        # --- STATE: TURN_AROUND ---
        elif self.current_state == "TURN_AROUND":
            # Blind Turn
            cmd.linear.x = 0.3
            cmd.angular.z = 1.0

        # --- STATE: BACKUP_PARK ---
        elif self.current_state == "BACKUP_PARK":
            if self.rear_tag:
                err_x = self.rear_tag.x
                cmd.linear.x = -0.15
                # Inverted steering for reverse
                cmd.angular.z = -2.0 * err_x 
            else:
                # Stop if rear tag not seen yet
                cmd.linear.x = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()