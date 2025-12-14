import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector

class RearAprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_rear_node')
        self.get_logger().info('--- Rear AprilTag Node Started ---')

        self.declare_parameter('show_vis', True)
        self.show_vis = self.get_parameter('show_vis').value
        
        self.declare_parameter('start_active', False)
        self.active = self.get_parameter('start_active').value

        self.bridge = CvBridge()
        self.command_sub = self.create_subscription(String, 'node_control', self.command_callback, 10)

        self.image_pub = self.create_publisher(Image, '/camera/rear/image_raw', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/rear/tag_pose', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/rear/debug_apriltag', 10)

        # --- CONFIG ---
        self.tag_size = 0.09 # 9cm Tag

        # Calibration
        self.camera_params = [590.9992, 622.7196, 395.7721, 370.0243]
        self.K = np.array([
            [590.9992,   0.0,      395.7721],
            [  0.0,    622.7196,   370.0243],
            [  0.0,      0.0,        1.0   ]
        ])
        self.D = np.array([[ 0.0449, -0.2814,  0.0504,  0.0144,  0.2261]])

        self.detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.033, self.timer_callback)
        
        if self.active: self.get_logger().info("Rear Camera Auto-Started")

    def command_callback(self, msg: String):
        data = msg.data.strip().split(':')
        if len(data) == 2 and data[0] == 'webcam_apriltag':
            self.active = (data[1] == '1')

    def timer_callback(self):
        if not self.active: return
        
        ret, raw_frame = self.cap.read()
        if not ret: return

        # Undistort
        h, w = raw_frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        frame = cv2.undistort(raw_frame, self.K, self.D, None, newcameramtx)

        # Detect
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=self.camera_params, 
            tag_size=self.tag_size
        )

        for det in detections:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "rear_cam_frame"
            
            # Position
            pose_msg.pose.position.x = det.pose_t[0][0]
            pose_msg.pose.position.y = det.pose_t[1][0]
            pose_msg.pose.position.z = det.pose_t[2][0]

            self.pose_pub.publish(pose_msg)

            # Debug
            if self.show_vis or self.debug_pub.get_subscription_count() > 0:
                corners = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [corners], True, (255, 0, 0), 2)
                cv2.circle(frame, (int(det.center[0]), int(det.center[1])), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Z: {det.pose_t[2][0]:.2f}m", (int(det.center[0]), int(det.center[1])-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(msg)
            self.debug_pub.publish(msg)
        except: pass

        if self.show_vis:
            display_frame = cv2.resize(frame, (0, 0), fx=2.0, fy=2.0)
            cv2.imshow("Rear AprilTag (9cm)", display_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RearAprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()