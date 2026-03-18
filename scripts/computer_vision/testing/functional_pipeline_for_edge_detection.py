import cv2
import numpy as np
import b_player_tracking_with_masking as pt_m
import player_tracking as pt

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Remember: 
# X positions of players are fixed (because they are fixed to a rod) 
# Y positions of players are variable 

img_received = False

rgb_img = np.zeros((360, 640, 3), dtype="uint8")


# get the image message 
def get_image(ros_img):
    global rgb_img
    global img_received
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8")
    # raise flag 
    img_received = True


# hsv filters might need to be adjusted based on lighting conditions
def normalize(image):
    img = image

    B, G, R = cv2.split(img)

    R = (((R - R.min()) / (R.max() - R.min())) * 255.0).astype(np.uint8)
    G = (((G - G.min()) / (G.max() - G.min())) * 255.0).astype(np.uint8)
    B = (((B - B.min()) / (B.max() - B.min())) * 255.0).astype(np.uint8)

    img = cv2.merge([B, G, R])

    return img


# hsv filters might need to be adjusted based on lighting conditions
def warp(image):
    img = image

    # corners of the field in the original image
    pts1 = np.float32([[23, 7], [619, 12], [19, 351], [618, 354]])

    # mapping to a new image
    pts2 = np.float32([[0, 0], [640, 0], [0, 360], [640, 360]])

    # find the corresponding transformation
    M = cv2.getPerspectiveTransform(pts1, pts2)

    # use the transformations to warp the initial image to a new plane/image
    dst = cv2.warpPerspective(img, M, (640, 360))

    return dst


def detect_corners(image, mask):
    img = image
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = mask

    corners = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=26,
        qualityLevel=0.01,
        minDistance=100,
        blockSize=3,
        useHarrisDetector=False,
        k=0.04
    )

    corners = np.intp(corners)  # Convert to integer coords
    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(img, (x, y), radius=3, color=(0, 255, 0), thickness=-1)

    return img, gray


# hsv filters might need to be adjusted based on lighting conditions
def green_filter(image, num):
    img = image

    wrap = num
    center = 70

    kernel = np.ones((9,9), np.uint8)

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Green has a Hue of ~60
    lower_green = np.array([center - wrap, 0, 0])
    upper_green = np.array([center + wrap, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    inverted_mask = cv2.bitwise_not(mask)
    inverted_mask = cv2.erode(inverted_mask, kernel, iterations=1)
    inverted_mask = cv2.dilate(inverted_mask, kernel, iterations=1)

    # WARNING: This majorly slows down the code!!
    # for i in range(hsv_masked.shape[0]):
    #    for j in range(hsv_masked.shape[1]):
    #        if hsv_masked[i][j][2] > 0:
    #            print(hsv_masked[i][j])

    return inverted_mask


# hsv filters might need to be adjusted based on lighting conditions
def hsv_filter(image, flag, num, size):
    img = image

    wrap_around = flag  # 0 or 1
    wrap = num

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if wrap_around == 1:
        # Define the range for the red color in HSV
        lower_red1 = np.array([0, 200, 50])
        upper_red1 = np.array([wrap, 255, 255])
        lower_red2 = np.array([180 - wrap, 200, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for the red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

    if wrap_around == 0:
        lower_red = np.array([0, 225, 200])
        upper_red = np.array([1, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

    # h, w = mask.shape[0], mask.shape[1]
    # large_mask = cv2.resize(mask, (w*2, h*2), interpolation=cv2.INTER_LINEAR)

    kernel = np.ones((size, size), np.uint8)
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)
    mask_erosion = cv2.erode(mask_dilation, kernel, iterations=1)

    # shrink_mask = cv2.resize(mask_erosion, (w, h), interpolation=cv2.INTER_LINEAR)

    # WARNING: This majorly slows down the code!!
    # for i in range(hsv_masked.shape[0]):
    #    for j in range(hsv_masked.shape[1]):
    #        if hsv_masked[i][j][2] > 0:
    #            print(hsv_masked[i][j])

    return mask_erosion


# hsv filters might need to be adjusted based on lighting conditions
def increase_brightness(image, num):
    img = image  # .copy()

    # img += num
    img = np.where(img + num < 255, img + num, 255)
    img = np.array(img, dtype=np.uint8)

    """
    B, G, R = cv2.split(img)

    def increase_value(input_value, increase_by):
        if input_value + increase_by > 255:
            return 255
        else:
            return input_value + increase_by

    new_B = [increase_value(v,num) for v in B.flatten()]
    # using dtype=uint8 as dtype
    new_B = np.array(new_B, dtype=np.uint8)
    new_B = new_B.reshape(B.shape)

    new_G = [increase_value(v,num) for v in G.flatten()]
    # using dtype=uint8 as dtype
    new_G = np.array(new_G, dtype=np.uint8)
    new_G = new_G.reshape(G.shape)

    new_R = [increase_value(v,num) for v in R.flatten()]
    # using dtype=uint8 as dtype
    new_R = np.array(new_R, dtype=np.uint8)
    new_R = new_R.reshape(R.shape)

    img = cv2.merge([new_B, new_G, new_R])
    """

    return img


# hsv filters might need to be adjusted based on lighting conditions
def blob_search(image, mask):
    def draw_points(image, points, color=(0, 0, 255)):

        for i in points:
            cv2.circle(image, i, 5, color, -1)

    """      
    def draw_lines(image, points): 

        distances = [] 

        # for every coordinate in red_points_ext 
        for i in range(len(points)): 
            # compare it to every other coordinate 
            for j in range(len(points)): 
                # make sure it is not the exact same coordinate 
                if points[j] != points[i]: 
                    # and check if the y-coordinate is within 10 pixels of each other 
                    if abs(points[i][1] - points[j][1]) <= 10: 
                        # then check if the x-coordinate is within 75 pixels of each other 
                        if abs(points[i][0] - points[j][0]) <= 75: 
                            # then calculate the distance between the two points 
                            dx = abs(points[i][0] - points[j][0]) 

						    # and draw a line between those two points
                            cv2.line(image, (points[i][0], points[i][1]), (points[j][0]+int(0.5*dx), points[j][1]),(255,0,0),5) 
                            distances.append(dx)
    """

    def draw_lines(image, points):

        # fixed positions of the rods along the x axis (will change with perspective transformations!)
        x_r_1 = 156
        x_r_2 = 380
        x_r_3 = 606

        for t in range(len(points)):

            # checking if point is along one of the player rods
            if (abs(points[t][0] - x_r_1) < 15) or (abs(points[t][0] - x_r_2) < 15) or (abs(points[t][0] - x_r_3) < 15):

                # finding out which player rod the point is a part of
                if (abs(points[t][0] - x_r_1) < 15):
                    x_r = x_r_1
                elif (abs(points[t][0] - x_r_2) < 15):
                    x_r = x_r_2
                elif (abs(points[t][0] - x_r_3) < 15):
                    x_r = x_r_3

                    # finding the point on the same y-plane
                for f in range(len(points)):
                    if points[f] != points[t]:

                        if (abs(points[t][1] - points[f][1]) <= 10) and (abs(points[f][0] - x_r) < 80):

                            dx = abs(points[t][0] - points[f][0])

                            # draw a rectangle based on the distance between the foot and torso that encompasses the whole player
                            if points[t][0] < points[f][0]:
                                # cv2.line(image, (points[t][0], points[t][1]), (points[f][0]+int(0.5*dx), points[f][1]),(255,0,0),5)
                                x1, y1, x2, y2 = (x_r - int(0.75 * dx), points[t][1] - 10, points[f][0] + int(0.5 * dx),
                                                  points[t][1] + 10)
                                cv2.rectangle(image, (x1, y1), (x2, y2), color=255, thickness=-1)
                            elif points[t][0] >= points[f][0]:
                                # cv2.line(image, (points[t][0], points[t][1]), (points[f][0]-int(0.5*dx), points[f][1]),(255,0,0),5)
                                x1, y1, x2, y2 = (x_r + int(0.75 * dx), points[t][1] - 10, points[f][0] - int(0.5 * dx),
                                                  points[t][1] + 10)
                                cv2.rectangle(image, (x1, y1), (x2, y2), color=255, thickness=-1)
                            # else:
                            #    x1, y1, x2, y2 = (x_r, points[t][1]-15, x_r, points[t][1] + 15)
                            #    cv2.rectangle(image, (x1, y1), (x2, y2), color=255, thickness=-1)

    img = image
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
        if M["m00"] != 0:  # m00 is the zeroeth moment and is the area of the contour
            # Calculate the center of the contour
            cX = int(M["m10"] / M[
                "m00"])  # m10 is the 1st order moment and is the sum of all of the x coordinates in the contour
            cY = int(M["m01"] / M[
                "m00"])  # m10 is the 1st order moment and is the sum of all of the y coordinates in the contour
            # Printing centers
            # print(f"Appx. Center of red object: ({cX}, {cY})")

            # Append the current center to the history of centers
            centers.append((cX, cY))

    draw_points(img, centers, color=(125, 255, 125))
    draw_lines(img, centers)

    return img

def edge_detection(image):

    img = image
    img = cv2.GaussianBlur(img, (7, 7), 0)

    ddepth = 3
    scale = 1
    delta = 0

    grad_x = cv2.Sobel(img, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
    grad_y = cv2.Sobel(img, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)

    abs_grad_x = cv2.convertScaleAbs(grad_x)
    abs_grad_y = cv2.convertScaleAbs(grad_y)

    grad = cv2.addWeighted(abs_grad_x, 1.0, abs_grad_y, 1.0, 0)

    return grad

if __name__ == '__main__':

    rospy.init_node('functional_pipeline', anonymous=True)

    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)

    pipe_pub = rospy.Publisher('/functional_pipeline', Image, queue_size=1)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():

        if img_received:

            frame = rgb_img
            frame = warp(frame)
            frame = green_filter(frame, 40)
            frame = edge_detection(frame)

            # Note: The original 'rod_mask' is a grayscale image (single channel).
            # If the publisher expects a color image (like the others), converting it to BGR8
            # will treat the single channel as all three channels (creating a visible grayscale image).
            pipe_msg = CvBridge().cv2_to_imgmsg(frame, encoding="8UC1")

            # img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1")
            pipe_pub.publish(pipe_msg)

        rate.sleep()
