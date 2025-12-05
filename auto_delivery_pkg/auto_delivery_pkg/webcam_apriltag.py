import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector

class LocalAprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_rear_node')
        self.get_logger().info('April Tag Node Started (Webcam - CPU Detection)')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/apriltag_image', 10)

        # --- 1. SETUP LOCAL DETECTOR ---
        self.get_logger().info("Initializing pupil_apriltags detector...")
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0, # 1.0 = Max accuracy, 2.0 = Faster (less range)
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # --- 2. SETUP WEBCAM ---
        self.init_webcam()

        # Timer loop (30 FPS target)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def init_webcam(self):
        # 0 is usually the default webcam. Change to 1 or 2 if using multiple cameras.
        self.cap = cv2.VideoCapture(0)

        # Set resolution to 640x480 for CPU performance balance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam! Is it plugged in?")
        else:
            self.get_logger().info("Webcam opened successfully (640x480).")

    def timer_callback(self):
        # 1. Get Frame from Webcam
        ret, frame = self.cap.read()

        if ret:
            # 2. Prepare for Detection
            # AprilTag algo requires Grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 3. Run Detection (ON CPU)
            detections = self.detector.detect(
                gray, 
                estimate_tag_pose=False, 
                camera_params=None, 
                tag_size=None
            )

            # 4. Process & Visualize Results
            for detection in detections:
                tag_id = detection.tag_id
                center = detection.center
                corners = detection.corners

                # --- Draw logic ---
                # Draw Center Dot (Red)
                cx, cy = int(center[0]), int(center[1])
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # Draw Corners (Green Box)
                pts = np.array(corners, dtype=np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                # Draw ID Text
                txt_x, txt_y = int(corners[0][0]), int(corners[0][1] - 10)
                cv2.putText(frame, f"ID: {tag_id}", (txt_x, txt_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Console Log
                self.get_logger().info(f"Tag Found: ID {tag_id} at ({cx}, {cy})")

            # 5. Show GUI
            cv2.imshow("Webcam Detection", frame)
            cv2.waitKey(1)

            # 6. Publish to ROS
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Publishing failed: {e}")
        else:
            self.get_logger().warn("Failed to read frame from webcam.")

def main(args=None):
    rclpy.init(args=args)
    node = LocalAprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Release the webcam resource
        if hasattr(node, 'cap'):
            node.cap.release()
        
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()