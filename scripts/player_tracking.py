import cv2
import numpy as np

def track_red(image):

    img = image

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range for the red color in HSV
    lower_red1 = np.array([0, 150, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 50])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for the red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Create a binary image where red is white and everything else is black
    binary_image = mask  # No need to invert the mask

    # Find contours in the binary image
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on size
    min_area = 100  # Minimum area of the contour
    max_area = 500  # Maximum area of the contour

    filtered_contours = [contour for contour in contours if min_area < cv2.contourArea(contour) < max_area]
    centers = []
    # Draw contours and find the center of each red object
    for contour in filtered_contours:
        # Calculate moments for each contour
        M = cv2.moments(contour)
        if M["m00"] != 0: #m00 is the zeroeth moment and is the area of the contour
            # Calculate the center of the contour
            cX = int(M["m10"] / M["m00"]) #m10 is the 1st order moment and is the sum of all of the x coordinates in the contour
            cY = int(M["m01"] / M["m00"]) #m10 is the 1st order moment and is the sum of all of the y coordinates in the contour
            # Printing centers
            #print(f"Appx. Center of red object: ({cX}, {cY})")

            # Append the current center to the history of centers
            centers.append((cX, cY))

    return centers

def track_blue(image):

    img = image

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range for the blue color in HSV
    lower_blue = np.array([90, 100, 0])
    upper_blue = np.array([135, 255, 192])

    # Create masks for the blue color
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Create a binary image where blue is white and everything else is black
    binary_image = mask  # No need to invert the mask

    # Find contours in the binary image
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on size
    min_area = 70  # Minimum area of the contour
    max_area = 500  # Maximum area of the contour

    filtered_contours = [contour for contour in contours if min_area < cv2.contourArea(contour) < max_area]
    centers = []
    # Draw contours and find the center of each blue object
    for contour in filtered_contours:
        # Calculate moments for each contour
        M = cv2.moments(contour)
        if M["m00"] != 0: #m00 is the zeroeth moment and is the area of the contour
            # Calculate the center of the contour
            cX = int(M["m10"] / M["m00"]) #m10 is the 1st order moment and is the sum of all of the x coordinates in the contour
            cY = int(M["m01"] / M["m00"]) #m10 is the 1st order moment and is the sum of all of the y coordinates in the contour
            # Printing centers
            #print(f"Appx. Center of blue object: ({cX}, {cY})")

            # Append the current center to the history of centers
            centers.append((cX, cY))

    return centers

def draw_points(image, points, color=(0, 0, 255)):

    for i in points:
        cv2.circle(image, i, 5, color, -1)

def main():

    # Read the image
    img = cv2.imread("C:\\Users\\benki\Documents\Education\Programming\Blue_Team_(Automated_Foosball_Table)\Blue_Team_Ex\Computer_Vision_Figures\\horizontal_and_angled_blue_players.jpg")
    img = cv2.resize(img, (640, 360))

    draw_points(img, track_red(img), (255, 0, 0))
    draw_points(img, track_blue(img), (0, 0, 255))

    # for i in track_blue(img):
    #     cv2.circle(img, i, 5, (0, 0, 255), -1)
    #
    # for i in track_red(img):
    #     cv2.circle(img, i, 5, (255, 0, 0), -1)

    # Display the original image with centers marked

    cv2.imshow('Original Image with Centers', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#main()