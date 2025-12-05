import cv2
import depthai as dai
import numpy as np
import time

# --- CONFIGURATION ---
# CHANGE THESE TO MATCH YOUR BOARD
CHESSBOARD_WIDTH = 10    # Number of INNER corners across
CHESSBOARD_HEIGHT = 7   # Number of INNER corners down
SQUARE_SIZE = 0.018     # Square size in meters (1.8cm)

# CRITERIA for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Calculating intrinsics... this might take a moment...

# ========================================
# CALIBRATION RESULTS
# ========================================
# Reprojection Error: 1.457074355933981
# --------------------
# COPY THIS INTO YOUR APRILTAG NODE:
# fx = 707.5638
# fy = 724.9585
# cx = 305.7333
# cy = 170.5174
# --------------------
# Full Matrix (K):
# [[707.56376429   0.         305.73330882]
#  [  0.         724.95853956 170.51737691]
#  [  0.           0.           1.        ]]
# Distortion Coeffs (D):
# [[ 2.17777158e-01 -1.59588428e+00 -5.50729301e-02 -3.74728018e-03
#    4.63829322e+00]]
# ========================================
# (donkey) pi@ucsdrobocar-148-14:~/projects/ros2_delivery_bot/utils $


# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHESSBOARD_HEIGHT * CHESSBOARD_WIDTH, 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_WIDTH, 0:CHESSBOARD_HEIGHT].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

def run_calibration():
    # --- 1. SETUP OAK-D PIPELINE ---
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setFps(30)
    
    # MUST MATCH YOUR APRILTAG NODE RESOLUTION
    cam_rgb.setPreviewSize(640, 480)
    
    # Auto-focus helps here, but hold still when capturing!
    cam_rgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)

    # --- 2. CAPTURE LOOP ---
    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        print("--- CALIBRATION STARTED ---")
        print(f"Searching for {CHESSBOARD_WIDTH}x{CHESSBOARD_HEIGHT} pattern.")
        print("Press 'c' to CAPTURE a frame.")
        print("Press 'q' to FINISH and CALCULATE.")
        print("Capture at least 15-20 images from different angles/distances.")

        count = 0

        while True:
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                display_frame = frame.copy()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Draw text
                cv2.putText(display_frame, f"Images: {count}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                cv2.imshow("Calibration", display_frame)

                key = cv2.waitKey(1)

                if key == ord('c'):
                    # Find the chess board corners
                    ret, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT), None)

                    if ret == True:
                        objpoints.append(objp)
                        
                        # Refine corner locations
                        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                        imgpoints.append(corners2)

                        # Flash detection on screen
                        cv2.drawChessboardCorners(display_frame, (CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT), corners2, ret)
                        cv2.imshow("Calibration", display_frame)
                        cv2.waitKey(500) # Pause to show capture
                        
                        count += 1
                        print(f"Captured Image {count}")
                    else:
                        print("Pattern not found! Hold steady or adjust lighting.")

                elif key == ord('q'):
                    break

        cv2.destroyAllWindows()

        # --- 3. CALCULATE ---
        if count < 10:
            print("Not enough images captured! Please restart and capture at least 10.")
            return

        print("\nCalculating intrinsics... this might take a moment...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        print("\n" + "="*40)
        print("CALIBRATION RESULTS")
        print("="*40)
        print(f"Reprojection Error: {ret}")
        print("-" * 20)
        print("COPY THIS INTO YOUR APRILTAG NODE:")
        print(f"fx = {mtx[0,0]:.4f}")
        print(f"fy = {mtx[1,1]:.4f}")
        print(f"cx = {mtx[0,2]:.4f}")
        print(f"cy = {mtx[1,2]:.4f}")
        print("-" * 20)
        print("Full Matrix (K):")
        print(mtx)
        print("Distortion Coeffs (D):")
        print(dist)
        print("="*40)

if __name__ == '__main__':
    run_calibration()