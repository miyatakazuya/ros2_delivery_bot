import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector
import math

class FrontAprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')
        self.get_logger().info('--- Front AprilTag Node (Calibrated) Started ---')

        self.declare_parameter('show_vis', True)
        self.show_vis = self.get_parameter('show_vis').value

        self.active = True
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(String, 'node_control', self.command_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/front/tag_pose', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/front/debug_apriltag', 10)

        # --- CALIBRATION DATA (From your update) ---
        self.tag_size = 0.06 # Note: Check if this is really 6cm. Standard paper tags are often 0.165m
        
        self.K = np.array([
            [707.5638, 0.0,      305.7333],
            [0.0,      724.9585, 170.5174],
            [0.0,      0.0,      1.0     ]
        ])
        self.D = np.array([0.2177, -1.5959, -0.0551, -0.0037, 4.6383])
        
        # [fx, fy, cx, cy]
        self.camera_params = [707.5638, 724.9585, 305.7333, 170.5174]

        self.detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)

    def command_callback(self, msg: String):
        data = msg.data.strip().split(':')
        if len(data) == 2 and data[0] == 'apriltag_node':
            self.active = (data[1] == '1')

    def image_callback(self, msg):
        if not self.active: return

        try:
            raw_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # 1. UNDISTORT
        h, w = raw_frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        frame = cv2.undistort(raw_frame, self.K, self.D, None, newcameramtx)

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
            pose_msg.header.frame_id = "oak_rgb_frame"
            
            # Position
            pose_msg.pose.position.x = det.pose_t[0][0]
            pose_msg.pose.position.y = det.pose_t[1][0]
            pose_msg.pose.position.z = det.pose_t[2][0]
            
            self.pose_pub.publish(pose_msg)

            # Debug
            if self.show_vis:
                corners = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(det.center[0]), int(det.center[1])), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Z: {det.pose_t[2][0]:.2f}m", (int(det.center[0]), int(det.center[1])-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        if self.show_vis:
            cv2.imshow("Front AprilTag (Calibrated)", frame)
            cv2.waitKey(1)
            
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = FrontAprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()