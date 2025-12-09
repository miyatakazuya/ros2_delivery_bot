import cv2
import numpy as np
import time

# --- CONFIGURATION ---
# CHANGE THESE TO MATCH YOUR BOARD
CHESSBOARD_WIDTH = 10    # Number of INNER corners across
CHESSBOARD_HEIGHT = 7    # Number of INNER corners down
SQUARE_SIZE = 0.018      # Square size in meters (1.8cm)

# WEBCAM CONFIG
# Try 0, 1, or 2 if you have multiple cameras plugged in (OAK-D usually takes a slot)
CAMERA_INDEX = 0         
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# CRITERIA for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHESSBOARD_HEIGHT * CHESSBOARD_WIDTH, 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_WIDTH, 0:CHESSBOARD_HEIGHT].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

def run_calibration():
    # --- 1. SETUP WEBCAM ---
    print(f"Opening Webcam (Index {CAMERA_INDEX})...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    # Set Resolution (MUST match your ROS node)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    # Check if opened
    if not cap.isOpened():
        print(f"ERROR: Could not open camera {CAMERA_INDEX}.")
        print("Try changing CAMERA_INDEX to 1 or 2.")
        return

    # Verify actual resolution
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera Resolution: {w}x{h}")

    # --- 2. CAPTURE LOOP ---
    print("--- CALIBRATION STARTED ---")
    print(f"Searching for {CHESSBOARD_WIDTH}x{CHESSBOARD_HEIGHT} pattern.")
    print("Press 'c' to CAPTURE a frame.")
    print("Press 'q' to FINISH and CALCULATE.")
    print("Capture at least 15-20 images from different angles/distances.")

    count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        display_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Draw text
        cv2.putText(display_frame, f"Images: {count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Webcam Calibration", display_frame)

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
                cv2.imshow("Webcam Calibration", display_frame)
                cv2.waitKey(500) # Pause to show capture
                
                count += 1
                print(f"Captured Image {count}")
            else:
                print("Pattern not found! Hold steady or adjust lighting.")

        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # --- 3. CALCULATE ---
    if count < 10:
        print("Not enough images captured! Please restart and capture at least 10.")
        return

    print("\nCalculating intrinsics... this might take a moment...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\n" + "="*40)
    print("CALIBRATION RESULTS (Rear Webcam)")
    print("="*40)
    print(f"Reprojection Error: {ret}")
    print("-" * 20)
    print("COPY THIS INTO 'webcam_apriltag.py':")
    print(f"self.camera_params = [{mtx[0,0]:.4f}, {mtx[1,1]:.4f}, {mtx[0,2]:.4f}, {mtx[1,2]:.4f}]")
    print("-" * 20)
    print("Full Matrix (K):")
    print(repr(mtx))
    print("Distortion Coeffs (D):")
    print(repr(dist))
    print("="*40)

if __name__ == '__main__':
    run_calibration()