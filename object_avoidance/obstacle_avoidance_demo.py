import cv2
import math
import numpy as np
from ultralytics import YOLO

MODEL_PATH = ".//best.pt"
WEBCAM_SOURCE = 1 

TARGET_CLASS_ID = 1      # Class ID for 'Obstacle'
MIN_CONFIDENCE = 0.25    

# --- TRIGGER THRESHOLDS ---
MIN_AREA_TRIGGER = 0.05  # Trigger if object is at least 5% of screen

# --- DYNAMIC MAPPING ---
# Range A: Start of avoidance (Object is somewhat visible)
RANGE_START_AREA = 0.05   # 5% screen coverage
RANGE_START_ANGLE = 70    # Gentle turn
RANGE_START_LEN = 200     # Shorter arrow

# Range B: Critical avoidance (Object is huge/close)
RANGE_END_AREA = 0.60     # 60% screen coverage (Lowered from 0.8)
RANGE_END_ANGLE = 5       # Sharp horizontal turn
RANGE_END_LEN = 500       # Very long arrow (Urgency)

ARROW_COLOR = (0, 0, 255) # Red
ARROW_THICKNESS = 6
# ==========================================

def map_range(value, in_min, in_max, out_min, out_max):
    """Maps value from input range to output range with clamping."""
    slope = (out_max - out_min) / (in_max - in_min)
    result = out_min + slope * (value - in_min)
    
    # Clamp result so it doesn't go crazy if we exceed limits
    if out_min < out_max:
        return max(out_min, min(result, out_max))
    else:
        return max(out_max, min(result, out_min))

def main():
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(WEBCAM_SOURCE)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
        
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    screen_area = width * height
    screen_center_x = width // 2

    print(f"Screen Resolution: {width}x{height}")
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(frame, conf=MIN_CONFIDENCE, verbose=False, iou=0.5)
        annotated_frame = results[0].plot()

        largest_obstacle = None
        max_area = 0

        # Find largest obstacle
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if cls_id == TARGET_CLASS_ID:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                box_w = x2 - x1
                box_h = y2 - y1
                area = box_w * box_h
                
                if area > max_area:
                    max_area = area
                    largest_obstacle = (x1, y1, x2, y2, box_w, box_h)

        if largest_obstacle:
            x1, y1, x2, y2, w, h = largest_obstacle
            obstacle_area_ratio = max_area / screen_area
            
            # Debug print to terminal so you can see the raw numbers
            # print(f"Ratio: {obstacle_area_ratio:.3f}")

            if obstacle_area_ratio >= MIN_AREA_TRIGGER:
                obj_center_x = x1 + (w // 2)

                # 1. Determine Direction
                # If center (within 50px margin) or left, go RIGHT.
                if obj_center_x <= (screen_center_x + 50): 
                    direction = "RIGHT"
                else:
                    direction = "LEFT"

                # 2. Calculate Dynamic Angle
                angle_deg = map_range(
                    obstacle_area_ratio, 
                    RANGE_START_AREA, RANGE_END_AREA, 
                    RANGE_START_ANGLE, RANGE_END_ANGLE
                )

                # 3. Calculate Dynamic Length
                arrow_len = map_range(
                    obstacle_area_ratio,
                    RANGE_START_AREA, RANGE_END_AREA,
                    RANGE_START_LEN, RANGE_END_LEN
                )

                # 4. Draw Arrow
                start_point = (width // 2, height - 50)
                
                if direction == "RIGHT":
                    theta_rad = math.radians(angle_deg)
                    end_x = start_point[0] + int(arrow_len * math.cos(theta_rad))
                    end_y = start_point[1] - int(arrow_len * math.sin(theta_rad))
                else: 
                    # Left is mirror of Right
                    theta_rad = math.radians(180 - angle_deg)
                    end_x = start_point[0] + int(arrow_len * math.cos(theta_rad))
                    end_y = start_point[1] - int(arrow_len * math.sin(theta_rad))

                # Overlay
                cv2.arrowedLine(annotated_frame, start_point, (end_x, end_y), ARROW_COLOR, ARROW_THICKNESS, tipLength=0.2)
                
                # On-screen Debug data
                label = f"AREA: {obstacle_area_ratio*100:.1f}% -> ANGLE: {angle_deg:.0f} deg"
                cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Dynamic Avoidance", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()