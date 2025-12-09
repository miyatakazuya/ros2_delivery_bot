import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
import os
import json

class OakPerceptionNode(Node):
    def __init__(self):
        super().__init__('oak_perception_node')
        self.get_logger().info('--- OAK Perception Node (Dual Stream) Started ---')

        # --- PARAMETERS ---
        self.declare_parameter('show_vis', True)
        self.show_vis = self.get_parameter('show_vis').value

        # --- CONTROL ---
        self.yolo_active = True 
        self.command_sub = self.create_subscription(String, 'node_control', self.command_callback, 10)

        # --- PUBLISHERS ---
        # 1. Raw Image (640x480) for AprilTag
        self.image_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        # 2. YOLO Data
        self.yolo_pub = self.create_publisher(String, '/perception/front/yolo_data', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/front/debug_yolo', 10)

        # --- CONFIG ---
        self.num_classes = 3
        self.label_map = ["Clear Box", "Red Box", "red box"]
        self.yolo_input_size = 640 # Square for NN
        
        pkg_share = get_package_share_directory('auto_delivery_pkg')
        self.blob_path = os.path.join(pkg_share, 'models', 'yolov8_n.blob')

        if not os.path.exists(self.blob_path):
            self.get_logger().error(f"Blob NOT found at: {self.blob_path}")
            return

        self.bridge = CvBridge()
        self.init_depthai()
        self.timer = self.create_timer(0.033, self.run_loop) 

    def init_depthai(self):
        pipeline = dai.Pipeline()

        # 1. RGB Camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)
        
        # --- DUAL STREAMS ---
        # Stream A: 640x640 for YOLO (Preview)
        cam_rgb.setPreviewSize(self.yolo_input_size, self.yolo_input_size)
        
        # Stream B: 640x480 for AprilTag Calibration (Video/ISP)
        cam_rgb.setVideoSize(640, 480) 

        # 2. Detection Network (Uses Preview)
        detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)
        detection_nn.setBlobPath(self.blob_path)
        detection_nn.setConfidenceThreshold(0.5) 
        detection_nn.setNumClasses(self.num_classes)
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([])      
        detection_nn.setAnchorMasks({}) 
        detection_nn.setIouThreshold(0.5) 
        detection_nn.input.setBlocking(False)
        
        # Link Preview -> NN
        cam_rgb.preview.link(detection_nn.input)

        # 3. Outputs
        # Output Video (640x480) to Host
        xout_video = pipeline.create(dai.node.XLinkOut)
        xout_video.setStreamName("video")
        cam_rgb.video.link(xout_video.input)

        # Output NN
        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        detection_nn.out.link(xout_nn.input)

        try:
            self.device = dai.Device(pipeline)
            self.q_video = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
            self.q_nn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize OAK-D: {e}")

    def command_callback(self, msg: String):
        data = msg.data.strip().split(':')
        if len(data) == 2 and data[0] in ['box_detection', 'oak_perception_node']:
            self.yolo_active = (data[1] == '1')

    def run_loop(self):
        if self.q_video is None: return

        # Get 640x480 frame for AprilTag
        in_video = self.q_video.tryGet()
        in_nn = self.q_nn.tryGet()

        if in_video is not None:
            frame_480 = in_video.getCvFrame() # 640x480
            
            # Publish RAW image (Calibrated size)
            raw_msg = self.bridge.cv2_to_imgmsg(frame_480, encoding="bgr8")
            self.image_pub.publish(raw_msg)

            # Process YOLO (Using the NN results which ran on the 640x640 preview)
            if self.yolo_active and in_nn is not None:
                detections = in_nn.detections
                yolo_data_list = []

                # Note: We draw on the 640x480 frame, so we must remap coordinates
                # YOLO coords are normalized (0-1). 
                # x * 640, y * 480 works because YOLO is robust to aspect ratio squishing.
                
                for det in detections:
                    x1 = int(det.xmin * 640)
                    y1 = int(det.ymin * 480)
                    x2 = int(det.xmax * 640)
                    y2 = int(det.ymax * 480)
                    
                    try:
                        label = self.label_map[det.label]
                    except:
                        label = str(det.label)

                    yolo_data_list.append({
                        "label": label,
                        "confidence": det.confidence,
                        "x_center": int((x1 + x2) / 2),
                        "y_center": int((y1 + y2) / 2),
                        "width": x2 - x1,
                        "height": y2 - y1
                    })

                    if self.show_vis:
                        cv2.rectangle(frame_480, (x1, y1), (x2, y2), (0, 0, 255), 2)

                if yolo_data_list:
                    json_str = json.dumps(yolo_data_list)
                    self.yolo_pub.publish(String(data=json_str))
                
                if self.show_vis:
                    cv2.imshow("OAK-D View", frame_480)
                    cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OakPerceptionNode()
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