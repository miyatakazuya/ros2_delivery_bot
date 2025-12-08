import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector

class FrontAprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')
        self.get_logger().info('--- Front AprilTag Node (Software) Started ---')

        # --- PARAMETERS ---
        self.declare_parameter('show_vis', True)
        self.show_vis = self.get_parameter('show_vis').value
        self.get_logger().info(f"Visualization Enabled: {self.show_vis}")

        self.active = True
        self.bridge = CvBridge()

        # --- SUBSCRIBERS ---
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/front/image_raw', 
            self.image_callback, 
            10
        )
        self.command_sub = self.create_subscription(String, 'node_control', self.command_callback, 10)

        # --- PUBLISHERS ---
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/front/tag_pose', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/front/debug_apriltag', 10)

        # --- DETECTOR ---
        self.detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)
        self.camera_params = [500.0, 500.0, 320.0, 320.0] 
        self.tag_size = 0.165

    def command_callback(self, msg: String):
        data = msg.data.strip().split(':')
        if len(data) == 2 and data[0] == 'apriltag_node':
            self.active = (data[1] == '1')

    def image_callback(self, msg):
        if not self.active: return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

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
            
            pose_msg.pose.position.x = det.pose_t[0][0]
            pose_msg.pose.position.y = det.pose_t[1][0]
            pose_msg.pose.position.z = det.pose_t[2][0]
            
            self.pose_pub.publish(pose_msg)

            # Debug Drawing
            if self.show_vis or self.debug_pub.get_subscription_count() > 0:
                corners = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(det.center[0]), int(det.center[1])), 5, (0, 0, 255), -1)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

        # --- TOGGLE VISUALIZATION ---
        if self.show_vis:
            cv2.imshow("Front AprilTag", frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FrontAprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()