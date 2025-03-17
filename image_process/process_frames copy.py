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

    def filter_green_color(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        return green_mask

    def region_selection(self, image):
        mask = np.zeros_like(image)
        ignore_mask_color = 255
        rows, cols = image.shape[:2]
        
        bottom_left = [cols * 0, rows * 0.95]
        bottom_left_up = [cols * 0, rows * 0.6]
        top_left = [cols * 0.4, rows * 0.4]
        bottom_right = [cols * 1, rows * 0.95]
        bottom_right_up = [cols * 1, rows * 0.6]
        top_right = [cols * 0.6, rows * 0.4]
        
        vertices = np.array([[bottom_left, bottom_left_up, top_left, top_right, bottom_right_up, bottom_right]], dtype=np.int32)
        
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(image, mask)
        return mask, masked_image

    def detect_lines(self, image):
        lines = cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=150)
        return lines

    def classify_and_average_lines(self, lines, image_height):
        left_slopes, left_intercepts = [], []
        right_slopes, right_intercepts = [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Flip y-coordinates to make bottom-left the origin
                y1, y2 = image_height - y1, image_height - y2

                if x1 == x2:  # Avoid vertical lines
                    continue
                m = (y2 - y1) / (x2 - x1)  # Slope
                b = y1 - m * x1  # Intercept
                
                if m < 0:  # Negative slope = left lane
                    left_slopes.append(m)
                    left_intercepts.append(b)
                else:  # Positive slope = right lane
                    right_slopes.append(m)
                    right_intercepts.append(b)

        avg_left_m = sum(left_slopes) / len(left_slopes) if left_slopes else None
        avg_left_b = sum(left_intercepts) / len(left_intercepts) if left_intercepts else None
        avg_right_m = sum(right_slopes) / len(right_slopes) if right_slopes else None
        avg_right_b = sum(right_intercepts) / len(right_intercepts) if right_intercepts else None

        # Compute x-intercepts (where y = 0)
        left_lane_pixel = -avg_left_b / avg_left_m if avg_left_m is not None else None
        right_lane_pixel = -avg_right_b / avg_right_m if avg_right_m is not None else None

        return (avg_left_m, avg_left_b, left_lane_pixel), (avg_right_m, avg_right_b, right_lane_pixel)


    def frame_processor(self, image):
        if image is None:
            print("Error: Could not load image.")
            return

        image_height = image.shape[0]  # Get image height for flipping y-coordinates

        green_mask = self.filter_green_color(image)
        edges = cv2.Canny(green_mask, 50, 150)
        mask, masked_edges = self.region_selection(edges)
        lines = self.detect_lines(masked_edges)

        (right_m, right_b, right_lane_pixel), (left_m, left_b, left_lane_pixel) = self.classify_and_average_lines(lines, image_height)

        if left_m is not None:
            print(f"Left Lane Equation (Bottom-Left Origin): y = {left_m:.2f}x + {left_b:.2f}")
            print(f"Left Lane X-Intercept: x = {left_lane_pixel:.2f} pixels")
        else:
            print("No left lane detected.")

        if right_m is not None:
            print(f"Right Lane Equation (Bottom-Left Origin): y = {right_m:.2f}x + {right_b:.2f}")
            print(f"Right Lane X-Intercept: x = {right_lane_pixel:.2f} pixels")
        else:
            print("No right lane detected.")


        if not (left_lane_pixel is None or right_lane_pixel is None):
            car_center_pixel = (image.shape[1])/2
            desired_pixel = (left_lane_pixel+right_lane_pixel)/2
            off_center_pixel = car_center_pixel-desired_pixel

            pixel_per_distance = (right_lane_pixel-left_lane_pixel)/(42) #full lane
            off_center_distance = (off_center_pixel/pixel_per_distance)+6 #for right

            #Printing left & right lane pixels
            print("Left Lane Pixel: %d\t| Right Lane Pixel: %d\t| Offcenter Pixels: %d\t| Offcenter Distance: %d"
            %(left_lane_pixel,right_lane_pixel,off_center_pixel,off_center_distance))

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
