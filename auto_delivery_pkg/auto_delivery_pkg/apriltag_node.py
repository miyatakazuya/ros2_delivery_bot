import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
from pupil_apriltags import Detector
from std_msgs.msg import String 

class LocalAprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')
        self.get_logger().info('April Tag Node Started (CPU Detection - Auto Focus)')

        # NEW: activation flag and node_control subscription
        self.active = False
        self.command_sub = self.create_subscription(
            String,
            'node_control',
            self.command_callback,
            10
        )

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/apriltag_image', 10)

        # --- 1. SETUP LOCAL DETECTOR ---
        # We run this on the Pi's CPU, not the Camera
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

        # --- 2. SETUP CAMERA STREAM ---
        self.init_depthai()

        # Timer loop (30 FPS target)
        self.timer = self.create_timer(0.033, self.timer_callback)
    
    # NEW: toggle based on "apriltag_node:1"/"apriltag_node:0"
    def command_callback(self, msg: String):
        command = msg.data.strip()
        try:
            target, state = command.split(':')
        except ValueError:
            self.get_logger().warn(f"Malformed command: '{command}'")
            return

        if target != 'apriltag_node':
            return

        if state not in ('0', '1'):
            self.get_logger().warn(f"Unknown state '{state}' for apriltag_node")
            return

        new_active = (state == '1')
        if new_active != self.active:
            self.active = new_active
            self.get_logger().info(
                f"Activation → {'ACTIVE' if self.active else 'IDLE'}"
            )
            
    def init_depthai(self):
        pipeline = dai.Pipeline()

        # Define RGB Camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)
        
        # --- FOCUS SETTINGS ---
        # REMOVED: Manual Focus Lock
        # ENABLED: Continuous Auto Focus
        cam_rgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)

        # RESIZE FOR CPU:
        # 640x480 is standard for Pi CPU processing
        cam_rgb.setPreviewSize(640, 480)

        # Output Stream
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        # Connect to Device
        self.device = dai.Device(pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        self.get_logger().info("OAK-D Streaming RGB (640x480). Auto-Focus Enabled.")

    def timer_callback(self):
        # NEW: only process when ACTIVE
        if not self.active:
            return

        # NEW: if no camera device, just skip
        if self.q_rgb is None:
            self.get_logger().warn("No RGB queue available (no OAK-D?). Skipping frame.")
            return
        
        # 1. Get Frame from Camera
        in_rgb = self.q_rgb.tryGet()

        if in_rgb is not None:
            frame = in_rgb.getCvFrame()

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
            cv2.imshow("Local CPU Detection", frame)
            cv2.waitKey(1)

            # 6. Publish to ROS
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(msg)
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = LocalAprilTagNode()
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