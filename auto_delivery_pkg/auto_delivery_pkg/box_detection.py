import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
import os
from std_msgs.msg import String

class BoxDetectionNode(Node):
    def __init__(self):
        super().__init__('box_detection')
        self.get_logger().info('--- Box Detection ROS 2 Node Started ---')

        # NEW: activation flag + node_control subscription
        self.active = False
        self.command_sub = self.create_subscription(
            String,
            'node_control',
            self.command_callback,
            10
        )

        # --- CONFIGURATION ---
        self.num_classes = 3
        self.label_map = ["Clear Box", "Red Box", "red box"]
        self.input_size = 640 
        
        # Locate the blob in the ROS install directory
        pkg_share = get_package_share_directory('auto_delivery_pkg')
        self.blob_path = os.path.join(pkg_share, 'models', 'yolov8_n.blob')

        # DEBUG: Print exactly where we are loading the blob from
        self.get_logger().info(f"LOADING BLOB FROM: {self.blob_path}")

        if not os.path.exists(self.blob_path):
            self.get_logger().error(f"Blob NOT found at: {self.blob_path}")
            return

        # ROS Setup
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/box_detection', 10)

        # Start DepthAI
        self.init_depthai()

        # Create a timer to poll the OAK-D (30 Hz)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def init_depthai(self):
        pipeline = dai.Pipeline()

        # 1. RGB Camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(20) 
        
        # PREVIEW MODE (Matches your working standalone script)
        # This crops the center 640x640 from the 1080p image.
        cam_rgb.setPreviewSize(self.input_size, self.input_size)

        # 2. Detection Network
        detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)
        detection_nn.setBlobPath(self.blob_path)
        detection_nn.setConfidenceThreshold(0.5) 
        detection_nn.setNumClasses(self.num_classes)
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([])      
        detection_nn.setAnchorMasks({}) 
        detection_nn.setIouThreshold(0.5) 

        # Link Preview directly to NN (No ImageManip node to cause distortion)
        cam_rgb.preview.link(detection_nn.input)

        # 3. Outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        detection_nn.out.link(xout_nn.input)
        
        # NEW: initialize handles
        self.device = None
        self.q_rgb = None
        self.q_nn = None

        # MODIFIED: Connect to device (wrapped so it’s safe w/o hardware)
        try:
            self.device = dai.Device(pipeline)
            self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.q_nn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            self.get_logger().info("Pipeline initialized. GUI Window should appear shortly.")
        except Exception as e:
            self.get_logger().error(f"Could not initialize DepthAI pipeline for box_detection: {e}")
            
    # NEW: handle "box_detection:1"/"box_detection:0"
    def command_callback(self, msg: String):
        command = msg.data.strip()
        try:
            target, state = command.split(':')
        except ValueError:
            self.get_logger().warn(f"Malformed command: '{command}'")
            return

        if target != 'box_detection':
            return

        if state not in ('0', '1'):
            self.get_logger().warn(f"Unknown state '{state}' for box_detection")
            return

        new_active = (state == '1')
        if new_active != self.active:
            self.active = new_active
            self.get_logger().info(
                f"Activation → {'ACTIVE' if self.active else 'IDLE'}"
            )

    def timer_callback(self):
        # NEW: only run this logic when ACTIVE
        if not self.active:
            return

        # NEW: if no queues (no OAK-D), just skip
        if self.q_rgb is None or self.q_nn is None:
            self.get_logger().warn("No DepthAI queues available; skipping detection.")
            return
        
        in_rgb = self.q_rgb.tryGet()
        in_nn = self.q_nn.tryGet()

        if in_rgb is not None:
            frame = in_rgb.getCvFrame()

            if in_nn is not None:
                detections = in_nn.detections
                for det in detections:
                    # Coordinates are normalized (0-1)
                    x1 = int(det.xmin * self.input_size)
                    y1 = int(det.ymin * self.input_size)
                    x2 = int(det.xmax * self.input_size)
                    y2 = int(det.ymax * self.input_size)

                    # Clamp
                    x1 = max(0, x1); y1 = max(0, y1)
                    x2 = min(self.input_size, x2); y2 = min(self.input_size, y2)

                    try:
                        label = self.label_map[det.label]
                    except:
                        label = str(det.label)

                    color = (0, 0, 255) if "Red" in label else (255, 0, 0)
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f"{label} {int(det.confidence * 100)}%", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # SHOW GUI
            cv2.imshow("Box Detection Debug", frame)
            cv2.waitKey(1)

            # Publish to ROS
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(msg)
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = BoxDetectionNode()
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