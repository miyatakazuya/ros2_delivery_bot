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
        self.get_logger().info('--- Rear AprilTag Node (Webcam) Started ---')

        # --- PARAMETERS ---
        self.declare_parameter('show_vis', True)
        self.show_vis = self.get_parameter('show_vis').value
        self.get_logger().info(f"Visualization Enabled: {self.show_vis}")

        self.active = False 
        self.bridge = CvBridge()
        self.command_sub = self.create_subscription(String, 'node_control', self.command_callback, 10)

        self.image_pub = self.create_publisher(Image, '/camera/rear/image_raw', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/perception/rear/tag_pose', 10)

        self.detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.camera_params = [600.0, 600.0, 320.0, 240.0]
        self.tag_size = 0.165 

        self.timer = self.create_timer(0.033, self.timer_callback)

    def command_callback(self, msg: String):
        data = msg.data.strip().split(':')
        if len(data) == 2 and data[0] == 'webcam_apriltag':
            self.active = (data[1] == '1')

    def timer_callback(self):
        if not self.active: return
        
        ret, frame = self.cap.read()
        if not ret: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

        for det in detections:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "rear_cam_frame"
            
            pose_msg.pose.position.x = det.pose_t[0][0]
            pose_msg.pose.position.y = det.pose_t[1][0]
            pose_msg.pose.position.z = det.pose_t[2][0]

            self.pose_pub.publish(pose_msg)

            # Debug Drawing
            if self.show_vis:
                corners = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [corners], True, (255, 0, 0), 2)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(msg)

        # --- TOGGLE VISUALIZATION ---
        if self.show_vis:
            cv2.imshow("Rear AprilTag", frame)
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