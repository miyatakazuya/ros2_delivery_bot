import depthai as dai
import cv2
import numpy as np
import os
import sys

class BoxDetection:
    def __init__(self):
        print('--- Standalone Box Detection Started ---')

        self.num_classes = 3
        self.label_map = ["Clear Box", "Red Box", "red box"]
        self.input_size = 640 
        
        self.blob_path = "../models/yolov8_n.blob" 

        if not os.path.exists(self.blob_path):
            print(f"ERROR: Blob not found at: {os.path.abspath(self.blob_path)}")
            print("Please put the .blob file in the same folder as this script.")
            sys.exit(1)

        self.run_depthai()

    def run_depthai(self):
        pipeline = dai.Pipeline()

        # 1. RGB Camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(20) 
        
        # PREVIEW MODE (Center Crop 640x640)
        # Note: This cuts off the left/right sides of the video!
        cam_rgb.setPreviewSize(self.input_size, self.input_size)

        # 2. Detection Network
        detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)
        detection_nn.setBlobPath(self.blob_path)
        detection_nn.setConfidenceThreshold(0.5) # Lowered slightly for testing
        detection_nn.setNumClasses(self.num_classes)
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([])      
        detection_nn.setAnchorMasks({}) 
        detection_nn.setIouThreshold(0.5) 

        # Link Preview directly to NN
        cam_rgb.preview.link(detection_nn.input)

        # 3. Outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        detection_nn.out.link(xout_nn.input)

        # Connect to device
        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

            print("YOLO Pipeline Running... Press 'q' to quit.")

            while True:
                in_rgb = q_rgb.tryGet()
                in_nn = q_nn.tryGet()

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

                            # Safety checks
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
                            
                            # Print to console so you know it's working
                            print(f"Det: {label} ({det.confidence:.2f})")

                    cv2.imshow("Debug View", frame)

                if cv2.waitKey(1) == ord('q'):
                    break
            
            cv2.destroyAllWindows()

if __name__ == '__main__':
    BoxDetection()