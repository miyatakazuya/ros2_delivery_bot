import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
from pupil_apriltags import Detector
import math

class AprilTagPoseNode(Node):
    def __init__(self):
        super().__init__('apriltag_pose_node')
        self.get_logger().info('April Tag Pose Node Started')
        
        self.tag_size = 0.06
        
        # YOUR CALIBRATION RESULTS
        self.K = np.array([
            [707.5638, 0.0,      305.7333],
            [0.0,      724.9585, 170.5174],
            [0.0,      0.0,      1.0     ]
        ])
        self.D = np.array([0.2177, -1.5959, -0.0551, -0.0037, 4.6383])
        
        # Camera Params list for AprilTag Detector [fx, fy, cx, cy]
        self.camera_params = [707.5638, 724.9585, 305.7333, 170.5174]

        # Init Bridge & Pubs
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/pose_debug', 10)
        
        # Car Control Publisher (Example)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Init Detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Start Camera
        self.init_depthai()
        self.timer = self.create_timer(0.033, self.timer_callback)

    def init_depthai(self):
        pipeline = dai.Pipeline()

        # RGB Camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)
        
        # 1. AUTO FOCUS (Essential for Pose)
        cam_rgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
        
        # 2. RESIZE for CPU (Must match calibration resolution 640x480)
        cam_rgb.setPreviewSize(640, 480)

        # Outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        self.device = dai.Device(pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.get_logger().info("Pipeline Running.")

    def rotation_matrix_to_euler(self, R):
        # Calculate rotation about X, Y, Z axis
        # For our car alignment, we mostly care about 'sy' (Yaw)
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.degrees([x, y, z]) # Returns [Pitch, Yaw, Roll] (approx)

    def timer_callback(self):
        in_rgb = self.q_rgb.tryGet()
        
        if in_rgb is not None:
            # 1. Get Frame
            raw_frame = in_rgb.getCvFrame()
            
            # 2. UNDISTORT (Important for accurate Pose)
            # This straightens curved lines so math works better
            h, w = raw_frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
            frame = cv2.undistort(raw_frame, self.K, self.D, None, newcameramtx)
            
            # 3. Detect
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(
                gray, 
                estimate_tag_pose=True, 
                camera_params=self.camera_params, 
                tag_size=self.tag_size
            )

            for det in detections:
                if det.pose_t is not None:
                    # Translation (Meters)
                    # t[0] = X (Left/Right) - Negative is Left
                    # t[1] = Y (Up/Down)    - Positive is Down
                    # t[2] = Z (Forward)    - Distance
                    t = det.pose_t.flatten()
                    
                    # Rotation
                    R = det.pose_R
                    euler = self.rotation_matrix_to_euler(R)
                    yaw_angle = euler[1] # Approximate Yaw

                    # --- CONTROL LOGIC INPUTS ---
                    lateral_error = t[0]  # Meters
                    distance = t[2]       # Meters
                    heading_error = yaw_angle # Degrees

                    # Visual Debug
                    self.draw_pose(frame, det, t, heading_error)
                    
                    # Console Log
                    print(f"ID:{det.tag_id} | Dist: {distance:.2f}m | LatErr: {lateral_error:.2f}m | Yaw: {heading_error:.1f}")

                    # --- SIMPLE ALIGNMENT LOGIC (Example) ---
                    # To align perpendicular: 
                    # 1. You want Lateral Error -> 0
                    # 2. You want Yaw -> 0 (Facing tag flat)
                    
                    twist = Twist()
                    if distance > 0.5:
                        # Simple P-Controller
                        steer = -1.0 * lateral_error # Steer into the error
                        twist.linear.x = 0.2
                        twist.angular.z = steer
                        # self.cmd_vel_pub.publish(twist)

            # Show GUI
            cv2.imshow("Pose Estimation", frame)
            cv2.waitKey(1)
            
            # Publish Debug Image
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(msg)
            except:
                pass

    def draw_pose(self, img, det, t, yaw):
        # Draw Box
        corners = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [corners], True, (0, 255, 0), 2)
        
        # Draw Text
        cx, cy = int(det.center[0]), int(det.center[1])
        cv2.putText(img, f"Dist: {t[2]:.2f}m", (cx, cy + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
        cv2.putText(img, f"Yaw: {yaw:.0f}", (cx, cy + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseNode()
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