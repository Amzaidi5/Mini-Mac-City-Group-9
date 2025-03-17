#!/usr/bin/env python

import numpy as np
#import pandas as pd
import cv2

import rospy
from scipy.cluster.hierarchy import dendrogram, linkage, fcluster
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class images:
    def __init__(self):
        self.left_sub = rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.frame_processor)
        
        #self.right_sub = rospy.Subscriber("/zed2/zed_node/right/image_rect_color", Image, self.right_image_processing)
        
        self.x_error_pub = rospy.Publisher('x_error', Float64, queue_size=3)
	self.bridge = CvBridge()

    def region_selection(self, image):
        """
        Determine and cut the region of interest in the input image.
        Parameters:
            image: we pass here the output from canny where we have 
            identified edges in the frame
        """
        # create an array of the same size as of the input image 
        mask = np.zeros_like(image) 
        # if you pass an image with more then one channel
        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_mask_color = (255,) * channel_count
        # our image only has one channel so it will go under "else"
        else:
            # color of the mask polygon (white)
            ignore_mask_color = 255
        # creating a polygon to focus only on the road in the picture
        # we have created this polygon in accordance to how the camera was placed
        rows, cols = image.shape[:2]

        #top left origin
        bottom_left  = [cols * 0, rows * 0.95]#[cols * 0.1, rows * 0.95]
        top_left     = [cols * 0.4, rows * 0.6]
        bottom_right = [cols * 1, rows * 0.95]#[cols * 0.9, rows * 0.95]
        top_right    = [cols * 0.6, rows * 0.6]
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        # filling the polygon with white color and generating the final mask
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        # performing Bitwise AND on the input image and mask to get only the edges on the road
        masked_image = cv2.bitwise_and(image, mask)

        #debug(mask)
        return masked_image

    def hough_transform(self, image):
        """
        Determine and cut the region of interest in the input image.
        Parameter:
            image: grayscale image which should be an output from the edge detector
        """
        # Distance resolution of the accumulator in pixels.
        rho = 1            
        # Angle resolution of the accumulator in radians.
        theta = np.pi/180
        # Only lines that are greater than threshold will be returned.
        threshold = 20    
        # Line segments shorter than that are rejected.
        minLineLength = 20
        # Maximum allowed gap between points on the same line to link them
        maxLineGap = 500    
        # function returns an array containing dimensions of straight lines 
        # appearing in the input image
        return cv2.HoughLinesP(image, rho = rho, theta = theta, threshold = threshold,
                            minLineLength = minLineLength, maxLineGap = maxLineGap)
        
    def average_slope_intercept(self, lines):
        """
        Find the slope and intercept of the left and right lanes of each image.
        Parameters:
            lines: output from Hough Transform
        """
        left_lines = [] #(slope, intercept)
        left_weights = [] #(length,)
        right_lines = [] #(slope, intercept)
        right_weights = [] #(length,)
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - (slope * x1)
                length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
                if slope < 0:
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))

        left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
        return left_lane, right_lane

    def pixel_points(self, y1, y2, line):
        """
        Converts the slope and intercept of each line into pixel points.
            Parameters:
                y1: y-value of the line's starting point.
                y2: y-value of the line's end point.
                line: The slope and intercept of the line.
        """
        if line is None:
            return None
        slope, intercept = line
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))

    def lane_lines(self, image, lines):
        """
        Create full lenght lines from pixel points.
            Parameters:
                image: The input test image.
                lines: The output lines from Hough Transform.
        """
        left_lane, right_lane = self.average_slope_intercept(lines)
        y1 = image.shape[0]
        y2 = y1 * 0.6
        left_line = self.pixel_points(y1, y2, left_lane)
        right_line = self.pixel_points(y1, y2, right_lane)
        return left_line, right_line

    """
    def draw_lane_lines(self, image, lines, color=[255, 0, 0], thickness=12):
        
        Draw lines onto the input image.
            Parameters:
                image: The input test image (video frame in our case).
                lines: The output lines from Hough Transform.
                color (Default = red): Line color.
                thickness (Default = 12): Line thickness. 
        
        line_image = np.zeros_like(image)
        for line in lines:
            if line is not None:
                cv2.line(line_image, *line, color, thickness)
        return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)

    def debug(image):
        image = cv2.resize(image, (960, 540))
        cv2.imshow("image",image)
    """

    def frame_processor(self, msg):
        """
        Process the input frame to detect lane lines.
        Parameters:
            image: image of a road where one wants to detect lane lines
            (we will be passing frames of video to this function)
        """
	image = self.bridge.imgmsg_to_cv2(msg, "bgr8")        
	grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # # Define the range of green color in HSV
        # lower_green = np.array([35, 40, 40])  # Adjust the values as needed for your image
        # upper_green = np.array([85, 255, 255])
        # # Create a mask for green color
        # grayscale = cv2.inRange(hsv, lower_green, upper_green)

        kernel_size = 5
        blur = cv2.GaussianBlur(grayscale, (kernel_size, kernel_size), 0)

        low_t = 50
        high_t = 150
        edges = cv2.Canny(blur, low_t, high_t)

        region = self.region_selection(edges)

        hough = self.hough_transform(region)
        left_line, right_line = self.lane_lines(image, hough)
        #result_overlay = self.draw_lane_lines(image, (left_line,right_line))


        #Taking care of None Type
        if not (left_line is None or right_line is None):
            left_line_pixel = left_line[0][0]
            right_line_pixel = right_line[0][0]

            car_center_pixel = (image.shape[1])/2
            desired_pixel = (left_line_pixel+right_line_pixel)/2
            off_center_pixel = abs(car_center_pixel-desired_pixel)

            pixel_per_distance = desired_pixel/(45+2) #full lane + half of green tape
            off_center_distance = (off_center_pixel/pixel_per_distance)#+6    #for left

            #if desired_pixel>off_center_pixel:
            #    off_center_distance = -off_center_distance

            #Printing left & right lane pixels
            #Note %d for offcenter distance that truncates 
            print("Left Lane Pixel: %d\t| Right Lane Pixel: %d\t| Offcenter Pixels: %d\t| Offcenter Distance: %.1f"
                %(left_line_pixel,right_line_pixel,off_center_pixel,off_center_distance))
            
            self.x_error_pub.publish(off_center_distance)

            return off_center_distance

def main(args=None):
    rospy.init_node('lane_detection', anonymous=True)
    im = images()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("stopping")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
