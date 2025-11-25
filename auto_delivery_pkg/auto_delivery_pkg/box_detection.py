import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import depthai as dai
import cv2
import numpy as np
import os

class BoxDetection(Node):
    def __init__(self):
        super().__init__('box_detection')
        self.get_logger().info('Box Detection Node Started')

        self.num_classes = 3   
        self.label_map = ["Clear Box", "Red Box", "red box"]
        self.input_size = 640  
        
        pkg_share = get_package_share_directory('auto_delivery_pkg')
        self.blob_path = os.path.join(pkg_share, 'models', 'yolov8_n.blob')

        if not os.path.exists(self.blob_path):
            self.get_logger().error(f"Blob not found at: {self.blob_path}")
            return

        self.run_depthai()

    def run_depthai(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(self.input_size, self.input_size) 
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(20) 

        detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)
        detection_nn.setBlobPath(self.blob_path)
        detection_nn.setConfidenceThreshold(0.5)
        detection_nn.setNumClasses(self.num_classes)
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([]) 
        detection_nn.setAnchorMasks({}) 
        detection_nn.setIouThreshold(0.5)

        
        cam_rgb.preview.link(detection_nn.input)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        detection_nn.out.link(xout_nn.input)

        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

            self.get_logger().info("YOLO Pipeline Running...")

            while rclpy.ok():
                in_rgb = q_rgb.tryGet()
                in_nn = q_nn.tryGet()

                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()

                    if in_nn is not None:
                        detections = in_nn.detections
                        for det in detections:
                            x1 = int(det.xmin * self.input_size)
                            y1 = int(det.ymin * self.input_size)
                            x2 = int(det.xmax * self.input_size)
                            y2 = int(det.ymax * self.input_size)

                            try:
                                label = self.label_map[det.label]
                            except:
                                label = str(det.label)

                            color = (0, 0, 255) if "Red" in label else (255, 0, 0)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame, f"{label} {int(det.confidence * 100)}%", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                            self.get_logger().info(f"Detected: {label} ({det.confidence:.2f})")

                    cv2.imshow("Box Detection Debug", frame)

                if cv2.waitKey(1) == ord('q'):
                    break
            
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = BoxDetection()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()