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
        self.get_logger().info('--- Rear AprilTag Node (Calibrated) Started ---')

        # --- PARAMETERS ---
        # This parameter allows you to toggle the X11 GUI on/off
        self.declare_parameter('show_vis', True)
        self.show_vis = self.get_parameter('show_vis').value
        self.get_logger().info(f"Visualization Enabled: {self.show_vis}")

        self.active = False 
        self.bridge = CvBridge()
        self.command_sub = self.create_subscription(String, 'node_control', self.command_callback, 10)

        self.image_pub = self.create_publisher(Image, '/camera/rear/image_raw', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/rear/tag_pose', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/rear/debug_apriltag', 10)

        # --- CALIBRATION DATA ---
        self.tag_size = 0.06 # 6cm Tag

        # [fx, fy, cx, cy]
        self.camera_params = [590.9992, 622.7196, 395.7721, 370.0243]

        self.K = np.array([
            [590.99916094,   0.        , 395.77207022],
            [  0.        , 622.7195623 , 370.0243088 ],
            [  0.        ,   0.        ,   1.        ]
        ])
        
        self.D = np.array([[ 0.04490013, -0.28135291,  0.05039422,  0.01435837,  0.22613974]])

        self.detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)
        
        # Webcam Setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.033, self.timer_callback)

    def command_callback(self, msg: String):
        data = msg.data.strip().split(':')
        if len(data) == 2 and data[0] == 'webcam_apriltag':
            self.active = (data[1] == '1')
            # Log status change for debugging
            #if self.active:
            #    self.get_logger().info("Rear Camera ACTIVATED")
            #else:
            #    self.get_logger().info("Rear Camera DEACTIVATED")

    def timer_callback(self):
        if not self.active: return
        
        ret, raw_frame = self.cap.read()
        if not ret: return

        # 1. UNDISTORT FRAME
        h, w = raw_frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        frame = cv2.undistort(raw_frame, self.K, self.D, None, newcameramtx)

        # 2. DETECT
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
            
            # Z is distance behind the robot
            pose_msg.pose.position.x = det.pose_t[0][0]
            pose_msg.pose.position.y = det.pose_t[1][0]
            pose_msg.pose.position.z = det.pose_t[2][0]

            self.pose_pub.publish(pose_msg)

            # Debug Drawing
            if self.show_vis or self.debug_pub.get_subscription_count() > 0:
                corners = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [corners], True, (255, 0, 0), 2)
                cv2.circle(frame, (int(det.center[0]), int(det.center[1])), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Z: {det.pose_t[2][0]:.2f}m", (int(det.center[0]), int(det.center[1])-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # Publish Debug Image
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(msg)
            self.debug_pub.publish(msg)
        except: pass

        # --- TOGGLE VISUALIZATION ---
        if self.show_vis:
            cv2.imshow("Rear AprilTag (Calibrated)", frame)
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
