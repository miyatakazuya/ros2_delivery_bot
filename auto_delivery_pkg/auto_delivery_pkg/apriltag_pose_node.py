import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

# ==========================================
# CONFIGURATION
# ==========================================
ENABLE_GUI = True  # Set to False if running headless
SCALE_FACTOR = 2.0 # Make window 2x bigger (1280x960)
# ==========================================

class RearPIDTuner(Node):
    def __init__(self):
        super().__init__('rear_pid_tuner')
        
        # --- LIVE TUNING PARAMETERS ---
        # Change these in terminal: ros2 param set /rear_pid_tuner k_p 1.0
        self.declare_parameter('k_p', 2.0)
        self.declare_parameter('target_speed', -0.15)
        self.declare_parameter('timeout', 0.2)

        # --- COMMUNICATIONS ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 1. Listen for the Tag Pose (Data)
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/perception/rear/tag_pose', 
            self.tag_callback, 
            10
        )

        # 2. Listen for the Debug Image (Visuals)
        if ENABLE_GUI:
            self.image_sub = self.create_subscription(
                Image,
                '/camera/rear/debug_apriltag',
                self.image_callback,
                10
            )

        self.bridge = CvBridge()

        # --- STATE ---
        self.latest_tag_x = None
        self.last_tag_time = 0.0
        self.msg_count = 0
        
        # For GUI display
        self.current_error = 0.0
        self.current_steer = 0.0
        self.is_active = False

        # Run control loop at 50Hz (0.02s)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Rear PID Tuner Started.")
        self.get_logger().info(f"GUI: {ENABLE_GUI} | Scale: {SCALE_FACTOR}x")

    def tag_callback(self, msg):
        self.latest_tag_x = msg.pose.position.x
        self.last_tag_time = time.time()
        self.msg_count += 1

    def image_callback(self, msg):
        if not ENABLE_GUI: return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # 1. Scale Up for visibility
        if SCALE_FACTOR != 1.0:
            frame = cv2.resize(frame, (0, 0), fx=SCALE_FACTOR, fy=SCALE_FACTOR)
        
        h, w = frame.shape[:2]
        center_x = w // 2

        # 2. Draw HUD (Heads Up Display)
        
        # STATUS
        status_color = (0, 255, 0) if self.is_active else (0, 0, 255)
        status_text = "ACTIVE" if self.is_active else "WAITING"
        cv2.putText(frame, f"MODE: {status_text}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

        # VALUES
        kp_val = self.get_parameter('k_p').value
        info = f"Err: {self.current_error:.3f}m | Steer: {self.current_steer:.2f} | Kp: {kp_val}"
        cv2.putText(frame, info, (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 3. VISUAL ERROR BAR
        # Yellow Line = Center, Red Dot = Tag Error
        # Scale: 100 pixels = 0.1m error (adjusted by scale factor)
        px_per_meter = 1000 * SCALE_FACTOR 
        err_px = int(self.current_error * px_per_meter)
        
        # Center Tick
        cv2.line(frame, (center_x, h-60), (center_x, h-20), (200, 200, 200), 2)
        
        # Error Line
        pt_center = (center_x, h - 40)
        pt_tag = (center_x + err_px, h - 40)
        cv2.line(frame, pt_center, pt_tag, (0, 255, 255), 3)
        cv2.circle(frame, pt_tag, 8, (0, 0, 255), -1)

        cv2.imshow("Rear PID Tuner", frame)
        cv2.waitKey(1)

    def control_loop(self):
        # Fetch live params
        k_p = self.get_parameter('k_p').value
        speed = self.get_parameter('target_speed').value
        timeout = self.get_parameter('timeout').value

        cmd = Twist()
        current_time = time.time()
        data_age = current_time - self.last_tag_time

        # Only drive if data is fresh
        if self.latest_tag_x is not None and data_age < timeout:
            self.is_active = True
            
            error_x = self.latest_tag_x
            
            # PID Calculation
            # REVERSE LOGIC: Usually, to move rear to the right (positive x), 
            # you steer right (negative z). 
            steer = -1.0 * k_p * error_x 

            cmd.linear.x = speed
            cmd.angular.z = steer
            
            # Save for GUI
            self.current_error = error_x
            self.current_steer = steer

            # Log occasionally
            if self.msg_count % 10 == 0:
                self.get_logger().info(
                    f"Age: {data_age:.3f}s | Err: {error_x:.2f}m | Steer: {steer:.2f}"
                )
        else:
            self.is_active = False
            self.current_error = 0.0
            self.current_steer = 0.0
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RearPIDTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist()) # Stop on exit
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()