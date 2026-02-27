import cv2
import numpy as np

# hsv filters might need to be adjusted based on lighting conditions
def track_red(image):

    img = image

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range for the red color in HSV
    lower_red1 = np.array([0, 120, 0])
    upper_red1 = np.array([30, 255, 255])
    lower_red2 = np.array([150, 120, 0])
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
    min_area = 75  # Minimum area of the contour
    max_area = 1000  # Maximum area of the contour

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

    return centers, binary_image

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

# the slice matrices are going to have to be re-adjusted for the real system
def rod_mask(img):

    # offset for video (might need to be adjusted)
    o = -20
    w = 7
	
    # blue slices
    b1 = (75 + o-10-w, 100 + o-10+w)
    b2 = (285 + o-w, 310 + o+w)
    b3 = (490 + o-w, 515 + o+w)

    # add minus 20 to r3 when using the image with one red player rod

    # red slices
    r1 = (75+100+5 + o-5-w, 100+100+5 + o-5+w)
    r2 = (285+100 + o-w, 310+100 + o+w)
    r3 = (490+100+5 + o-w, 515+100+5 + o+w)

    # fills
    f1 = (0, b1[0])
    f2 = (b1[1], r1[0])
    f3 = (r1[1], b2[0])
    f4 = (b2[1], r2[0])
    f5 = (r2[1], b3[0])
    f6 = (b3[1], r3[0])
    f7 = (r3[1], 640)

    # blue regions of interest
    b_roi_1 = img[:, b1[0]:b1[1]]
    b_roi_2 = img[:, b2[0]:b2[1]]
    b_roi_3 = img[:, b3[0]:b3[1]]

    # red regions of interest
    r_roi_1 = img[:, r1[0]:r1[1]]
    r_roi_2 = img[:, r2[0]:r2[1]]
    r_roi_3 = img[:, r3[0]:r3[1]]

    fill_1 = np.zeros((360, f1[1] - f1[0], 3), dtype=np.uint8)
    fill_2 = np.zeros((360, f2[1] - f2[0], 3), dtype=np.uint8)
    fill_3 = np.zeros((360, f3[1] - f3[0], 3), dtype=np.uint8)
    fill_4 = np.zeros((360, f4[1] - f4[0], 3), dtype=np.uint8)
    fill_5 = np.zeros((360, f5[1] - f5[0], 3), dtype=np.uint8)
    fill_6 = np.zeros((360, f6[1] - f6[0], 3), dtype=np.uint8)
    fill_7 = np.zeros((360, f7[1] - f7[0], 3), dtype=np.uint8)

    # stack regions of interest to match the dimensions of the original image
    h_stack = np.hstack((fill_1, b_roi_1, fill_2, r_roi_1, fill_3, b_roi_2, fill_4, r_roi_2, fill_5, b_roi_3, fill_6, r_roi_3, fill_7))

    return h_stack

def main():

    # Read the image
    img = cv2.imread("C:\\Users\\benki\Documents\Education\Programming\Blue_Team_(Automated_Foosball_Table)\Blue_Team_Ex\Computer_Vision_Figures\\horizontal_and_angled_blue_players.jpg")
    #img = cv2.imread("C:\\Users\\benki\Documents\Education\Programming\Blue_Team_(Automated_Foosball_Table)\Blue_Team_Ex\Computer_Vision_Figures\\front horizontal red players.jpg")

    img = cv2.resize(img, (640, 360))
    img = rod_mask(img)

    # Detect blue players
    for i in track_blue(img):
        cv2.circle(img, i, 5, (0, 0, 255), -1)

    # Detect red players
    for i in track_red(img):
        cv2.circle(img, i, 5, (255, 0, 0), -1)

    # Display the original image with centers marked
    cv2.imshow('Original Image with Centers', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#main()
