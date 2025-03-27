#!/usr/bin/env python

import numpy as np
from sklearn.cluster import KMeans
import cv2

import rospy
import tf
from scipy.cluster.hierarchy import dendrogram, linkage, fcluster
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class left_images:
    def __init__(self):
        self.left_sub = rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.frame_processor)
        self.x_error_left_pub = rospy.Publisher('x_error_left', Float64, queue_size=3)
        self.bridge = CvBridge()

    def filter_green_color(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        return green_mask

    def region_selection(image):
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

    def detect_lines(image):
        lines = cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=50, minLineLength=200, maxLineGap=150)
        return lines

    def classify_and_average_lines(lines, image_height):
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

        if (not left_intercepts) or (not right_intercepts):     #empty list
            return (None, None, None), (None, None, None)


        # Combine intercepts
        combined_intercepts = left_intercepts + right_intercepts
        combined_slopes = left_slopes + right_slopes

        # Convert the intercepts into a numpy array for clustering
        intercepts_array = np.array(combined_intercepts).reshape(-1, 1)

        # Perform KMeans clustering (3 clusters)
        kmeans = KMeans(n_clusters=3, random_state=42)
        clusters = kmeans.fit_predict(intercepts_array)

        # Create a dictionary to store the clusters and their corresponding values
        clustered_data = {0: {'intercepts': [], 'slopes': []},
                        1: {'intercepts': [], 'slopes': []},
                        2: {'intercepts': [], 'slopes': []}}

        # Fill the clusters with intercepts and slopes
        for i, cluster in enumerate(clusters):
            clustered_data[cluster]['intercepts'].append(combined_intercepts[i])
            clustered_data[cluster]['slopes'].append(combined_slopes[i])

        # Calculate the average slope and intercept for each cluster
        cluster_averages = {}

        for cluster_num, data in clustered_data.items():
            avg_slope = np.mean(data['slopes'])
            avg_intercept = np.mean(data['intercepts'])
            cluster_averages[cluster_num] = {'avg_slope': avg_slope, 'avg_intercept': avg_intercept}

        # Find the clusters with the highest and lowest average slope
        highest_slope_cluster = max(cluster_averages, key=lambda x: cluster_averages[x]['avg_slope'])
        lowest_slope_cluster = min(cluster_averages, key=lambda x: cluster_averages[x]['avg_slope'])
        
        avg_left_m = cluster_averages[highest_slope_cluster]['avg_slope']
        avg_left_b = cluster_averages[highest_slope_cluster]['avg_intercept']
        avg_right_m = cluster_averages[lowest_slope_cluster]['avg_slope']
        avg_right_b = cluster_averages[lowest_slope_cluster]['avg_intercept']

        # Compute x-intercepts (where y = 0)
        left_lane_pixel = -avg_left_b / avg_left_m if avg_left_m is not None else None
        right_lane_pixel = -avg_right_b / avg_right_m if avg_right_m is not None else None

        return (avg_left_m, avg_left_b, left_lane_pixel), (avg_right_m, avg_right_b, right_lane_pixel)
    
    def frame_processor(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")    
    
        if image is None:
            print("Error: Could not load image.")
            return

        image_height = image.shape[0]  # Get image height for flipping y-coordinates

        image = cv2.GaussianBlur(image, (15, 15), 0)
        green_mask = self.filter_green_color(image)
        
        edges = cv2.Canny(green_mask, 255, 255)
        mask, masked_edges = self.region_selection(edges)
        lines = self.detect_lines(masked_edges)

        (left_m, left_b, left_lane_pixel), (right_m, right_b, right_lane_pixel) = self.classify_and_average_lines(lines, image_height)

        if left_m is None:
            print("No left lane detected")
        if right_m is None:
            print("No right lane detected")

        if not (left_lane_pixel is None or right_lane_pixel is None):
            car_center_pixel = (image.shape[1])/2
            desired_pixel = (left_lane_pixel+right_lane_pixel)/2
            off_center_pixel = car_center_pixel-desired_pixel

            pixel_per_distance = (right_lane_pixel-left_lane_pixel)/(42) #full lane
            off_center_distance = (off_center_pixel/pixel_per_distance)+6

            #Printing left & right lane pixels
            print("Left Lane Pixel: %d\t| Right Lane Pixel: %d\t| Offcenter Pixels: %d\t| Offcenter Distance: %f"
            %(left_lane_pixel,right_lane_pixel,off_center_pixel,off_center_distance))

            self.x_error_left_pub.publish(off_center_distance)


        return off_center_distance


def main(args=None):
    rospy.init_node('lane_detection', anonymous=True)
    im = left_images()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("stopping")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass