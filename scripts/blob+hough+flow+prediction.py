import cv2
import numpy as np
import math

def create_white_mask_hsv(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 70, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 1))
    blue_mask_expanded = cv2.dilate(blue_mask, horizontal_kernel, iterations=1)

    white_mask_cleaned = cv2.bitwise_and(white_mask, cv2.bitwise_not(blue_mask_expanded))

    return white_mask_cleaned

def filter_with_opening_and_growth(binary_mask, kernel_size=7, growth_iterations=2):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    opened_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
    growth_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    grown_mask = cv2.dilate(opened_mask, growth_kernel, iterations=growth_iterations)
    return grown_mask

def detect_blobs(mask, frame):
    detections = 0
    position = None

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 3.14 * 12 * 12
    params.maxArea = 3.14 * 20 * 20
    params.filterByCircularity = True
    params.minCircularity = 0.8
    params.maxCircularity = 1.0
    params.filterByConvexity = True
    params.minConvexity = 0.85
    params.maxConvexity = 1.0
    params.filterByInertia = True
    params.minInertiaRatio = 0.5

    params.filterByColor = True
    params.blobColor = 255

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(mask)

    if keypoints:
        largest_keypoint = max(keypoints, key=lambda kp: kp.size)
        position = (int(largest_keypoint.pt[0]), int(largest_keypoint.pt[1]))
        detections = 1

    return position, detections

def detect_hough_circles(mask, frame):
    detections = 0
    position = None

    blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2)
    circles = cv2.HoughCircles(blurred_mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                               param1=50, param2=15, minRadius=10, maxRadius=20)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            radius = i[2]
            if 10 <= radius <= 20:
                position = (i[0], i[1])
                detections = 1
                break

    return position, detections

# Clamp position within the defined gameboard bounds
def clamp_position_to_board(position, board_bounds):
    x_min, y_min, x_max, y_max = board_bounds
    x = max(x_min, min(x_max, position[0]))
    y = max(y_min, min(y_max, position[1]))
    return (x, y)

def main():
    cap = cv2.VideoCapture('C:/Users/benki/Documents/School/Programming/Blue Team Revamped/Videos/17output.mp4')

    if not cap.isOpened():
        print("Error: Cannot open video file")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 639)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 359)

    board_bounds = (20, 20, 619, 339)  # Define the board bounds manually (x_min, y_min, x_max, y_max)

    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting...")
        return

    last_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    last_position = None
    last_velocity = (0, 0)
    total = 0
    captured = 0
    first_cap = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting...")
            break

        velocity = (0, 0)

        white_mask = create_white_mask_hsv(frame)
        filtered_mask = filter_with_opening_and_growth(white_mask, kernel_size=7, growth_iterations=2)
        filtered_bgr = cv2.cvtColor(filtered_mask, cv2.COLOR_GRAY2BGR)

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        position, detections = detect_blobs(filtered_mask, frame)
        detection_method = "Blob Detection"

        if detections == 0:
            position, detections = detect_hough_circles(filtered_mask, frame)
            detection_method = "Hough Circle Detection"

        if detections == 0 and last_position is not None:
            p0 = np.array([[last_position]], dtype=np.float32)
            p1, st, err = cv2.calcOpticalFlowPyrLK(last_gray, frame_gray, p0, None)
            if st[0][0] == 1:
                flow_pos = (int(p1[0][0][0]), int(p1[0][0][1]))
                pred_pos = (last_position[0] + last_velocity[0], last_position[1] + last_velocity[1])
                vx, vy = last_velocity
                speed = math.hypot(vx, vy)
                if speed > 1000000:
                    position = (int(0.005 * flow_pos[0] + 0.995 * pred_pos[0]), int(0.005 * flow_pos[1] + 0.995 * pred_pos[1]))
                else:
                    position = (int(1 * flow_pos[0] + 0 * pred_pos[0]), int(1 * flow_pos[1] + 0 * pred_pos[1]))
                detections = 2
                detection_method = "Optical Flow + Prediction"
            else:
                position = (last_position[0] + last_velocity[0], last_position[1] + last_velocity[1])
                detections = 3
                detection_method = "Fallback Prediction"

        if position is not None:
            position = clamp_position_to_board(position, board_bounds)

        if position is not None and last_position is not None:
            velocity = (position[0] - last_position[0], position[1] - last_position[1])

        frame_with_detection = frame.copy()
        if position is not None:
            if detection_method == "Blob Detection":
                cv2.circle(frame_with_detection, position, 15, (0, 255, 0), 2)
            elif detection_method == "Hough Circle Detection":
                cv2.circle(frame_with_detection, position, 15, (0, 255, 0), 2)
            elif detection_method == "Optical Flow + Prediction":
                cv2.circle(frame_with_detection, position, 15, (255, 0, 255), 2)
            elif detection_method == "Fallback Prediction":
                cv2.circle(frame_with_detection, position, 15, (0, 165, 255), 2)

        cv2.rectangle(frame_with_detection, (board_bounds[0], board_bounds[1]), (board_bounds[2], board_bounds[3]), (0, 255, 255), 2)

        combined = np.hstack((frame_with_detection, filtered_bgr))

        cv2.imshow('Detection (Left) | Processed (Right)', combined)

        if detections in [1, 2, 3]:
            if first_cap == 0:
                captured += 1
                total += 1
                first_cap = 1
            else:
                total += 1
                captured += 1
        else:
            if first_cap == 1:
                total += 1

        if total != 0:
            print(f"Detection Rate: {captured / total:.2f} | Method Used: {detection_method}")
        else:
            print("No capture yet")

        last_velocity = velocity if position != last_position else last_velocity
        last_position = position
        last_gray = frame_gray.copy()

        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
