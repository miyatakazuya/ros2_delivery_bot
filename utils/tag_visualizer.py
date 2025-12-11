import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector
import math

class PoseDemoNode(Node):
    def __init__(self):
        super().__init__('pose_demo_node')
        
        # --- CONFIG ---
        self.declare_parameter('tag_size', 0.12) # Default to your scaled size
        self.tag_size = self.get_parameter('tag_size').value
        
        # Calibration (Your OAK-D Matrix)
        self.K = np.array([
            [707.5638, 0.0,      305.7333],
            [0.0,      724.9585, 170.5174],
            [0.0,      0.0,      1.0     ]
        ])
        self.D = np.array([0.2177, -1.5959, -0.0551, -0.0037, 4.6383])

        # --- SETUP ---
        self.bridge = CvBridge()
        self.detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)
        
        # Subscriber (Listen to OAK-D stream)
        self.image_sub = self.create_subscription(
            Image, '/camera/front/image_raw', self.image_callback, 10
        )
        
        self.get_logger().info("Pose Demo Node Started. Waiting for video...")

    def get_yaw_from_R(self, R):
        # Extract Yaw from Rotation Matrix
        yaw = math.atan2(R[0, 2], R[2, 2])
        return np.degrees(yaw)

    def draw_top_down_view(self, x_dist, z_dist, yaw):
        """
        Creates a 'Radar' style plot.
        Car is fixed at Bottom Center.
        """
        # Canvas Settings
        H, W = 600, 500
        canvas = np.zeros((H, W, 3), dtype=np.uint8) # Black background
        
        # Scale: 100 pixels = 1 meter
        scale = 100 
        
        # 1. Draw Grid (Every 0.5m)
        for i in range(0, H, int(0.5 * scale)):
            color = (50, 50, 50) # Dark Grey
            cv2.line(canvas, (0, H-i), (W, H-i), color, 1)
            cv2.putText(canvas, f"{i/scale}m", (10, H-i-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Vertical Center Line
        cv2.line(canvas, (W//2, 0), (W//2, H), (50, 50, 50), 1)

        # 2. Draw Car (Triangle at Bottom Center)
        car_x = W // 2
        car_y = H - 50
        pts = np.array([
            [car_x, car_y - 20],  # Nose
            [car_x - 15, car_y + 20], # Rear Left
            [car_x + 15, car_y + 20]  # Rear Right
        ], np.int32)
        cv2.fillPoly(canvas, [pts], (0, 255, 255)) # Yellow Car
        cv2.putText(canvas, "CAR", (car_x-15, car_y+40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 3. Draw Tag
        # Coordinate Logic:
        # ROS X is Left/Right (Negative Right?). 
        # AprilTag X: Right is Positive. 
        # Screen X: Right is Positive.
        # So Screen_X = Center + (Tag_X * Scale)
        
        tag_screen_x = int(car_x + (x_dist * scale))
        tag_screen_z = int(car_y - (z_dist * scale)) # Z goes 'Up' the screen

        # Draw Tag Body (Green Box)
        # We rotate the box to match Yaw
        rect_w = 40 # px width
        rect_h = 10 # px thickness
        
        rect = ((tag_screen_x, tag_screen_z), (rect_w, rect_h), yaw)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(canvas, [box], 0, (0, 255, 0), -1)

        # Draw Line connecting Car to Tag
        cv2.line(canvas, (car_x, car_y), (tag_screen_x, tag_screen_z), (0, 100, 0), 1)

        # 4. Draw Info Text
        text = [
            f"DIST (Z): {z_dist:.2f} m",
            f"ERR  (X): {x_dist:.2f} m",
            f"YAW     : {yaw:.0f} deg"
        ]
        
        for i, line in enumerate(text):
            cv2.putText(canvas, line, (20, 30 + (i*25)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return canvas

    def image_callback(self, msg):
        try:
            raw_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # 1. Undistort (Standard Pipeline)
        h, w = raw_frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        frame = cv2.undistort(raw_frame, self.K, self.D, None, newcameramtx)

        # Extract NEW intrinsics
        fx = newcameramtx[0, 0]
        fy = newcameramtx[1, 1]
        cx = newcameramtx[0, 2]
        cy = newcameramtx[1, 2]
        current_params = [fx, fy, cx, cy]

        # 2. Detect
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=current_params,
            tag_size=self.tag_size
        )

        # 3. Viz Logic
        if len(detections) > 0:
            # Visualize the first tag found
            det = detections[0]
            
            x = det.pose_t[0][0]
            z = det.pose_t[2][0]
            yaw = self.get_yaw_from_R(det.pose_R)
            
            # Generate Radar Plot
            radar_view = self.draw_top_down_view(x, z, yaw)
            cv2.imshow("Pose Radar", radar_view)
            
        else:
            # Show empty radar if no tag
            empty_view = self.draw_top_down_view(0, 0, 0)
            cv2.putText(empty_view, "NO TAG DETECTED", (150, 300), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow("Pose Radar", empty_view)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PoseDemoNode()
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