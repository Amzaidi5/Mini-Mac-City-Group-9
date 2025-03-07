#!/usr/bin/env python
#Uses modified code from lane_detection_v1

import rospy
import cv2
import numpy as np

from scipy.cluster.hierarchy import dendrogram, linkage, fcluster
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# Image resolution
HEIGHT = 360
WIDTH = 640

# Offsets to account for position of the left lens in the ZED camera. Values based on experimental results.
X_OFFSET = -55
PHI_OFFSET = 12

# Region of interest mask coordinates
TOP_LEFT_X = 0
BOTTOM_LEFT_X = 0
TOP_RIGHT_X = 150
BOTTOM_RIGHT_X = 0
TOP_Y = 4.50*HEIGHT/7
BOTTOM_Y = 6.50*HEIGHT/7

class images:
    def __init__(self):
	#Subscribers        
	self.sub = rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color", Image, self.imageProcessing)
	#self.cap = cv2.VideoCapture('/home/nvidia/Documents/ZED/Run3_trimmed.mp4')
	#Publishers
	self.x_error_left_pub = rospy.Publisher('x_error_left', Float64, queue_size=3)
	self.phi_error_left_pub = rospy.Publisher('phi_error_left', Float64, queue_size=3)

        self.bridge = CvBridge()
        self.last_lanes = [-999, -999]

    def imageProcessing(self, msg, verbose=False, test=False):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") #Frame from ZED camera topic
	except CvBridgeError as e:
		rospy.loginfo("cv conversion error: " + str(e))
        
	cv_canny = self.canny(cv_image)
        cv_masked = self.addmask(cv_canny)
        lane_edges = self.find_edges(cv_masked)
        [cv_lines, x_error, phi_error] = self.curved_lines(
            cv_image, lane_edges, True)
        cv_final = cv2.addWeighted(cv_lines, 0.8, cv_image, 1, 1)
        #self.show_vis(cv_masked, cv_final)

        try:
       	    #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_final, "bgr8"))
	    self.x_error_left_pub.publish(x_error)
	    self.phi_error_left_pub.publish(phi_error)

        except CvBridgeError as e:
            rospy.loginfo("publish error: " + str(e))

 #---Sub Functions---#
    def canny(self, frame):	
	hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_green = np.array([40,50,0])
	upper_green = np.array([90,255,255])
	green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
	color_isolated = cv2.bitwise_and(frame, frame, mask=green_mask)
	#cv2.imshow('isolated', color_isolated)
	#cv2.waitKey(1)
	gray_frame = cv2.cvtColor(color_isolated, cv2.COLOR_RGB2GRAY)
	
        blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
	ret, thresh_frame = cv2.threshold(blurred_frame, 60, 255, cv2.THRESH_BINARY)
	#thresh_frame = cv2.adaptiveThreshold(blurred_frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
	#cv2.imshow('thresh', thresh_frame)
	#cv2.waitKey(1)	
	return cv2.Canny(thresh_frame, 80, 125)

    def addmask(self, frame):
        h, w = frame.shape[:2]
        #triangle = np.array([[(int(10), int(6*h/7)), (int(w-10), int(6*h/7)),
        #                    (int(w-10), int(5*h/7)), (int(10), int(5*h/7))]])
	triangle = np.array([[(int(BOTTOM_LEFT_X), int(BOTTOM_Y)), (int(w-BOTTOM_RIGHT_X), int(BOTTOM_Y)),
                            (int(w-TOP_RIGHT_X), int(TOP_Y)), (int(TOP_LEFT_X), int(TOP_Y))]])
        mask1 = np.zeros_like(frame)
        cv2.fillPoly(mask1, triangle, 255)
        frame = cv2.bitwise_and(frame, mask1)
        return frame

    def find_edges(self, frame):
        line_frame = np.zeros_like(frame)
        edges = []

        np_frame = np.array(frame)

        (x, y) = np.nonzero(np_frame)
        temp_edges = zip(y, x)

        if temp_edges != []:
            cluster = linkage(temp_edges,
                              method='single', metric='euclidean')
            group = fcluster(cluster, 5, criterion='distance')
            edges = [[[], []] for idx in range(max(group))]

            for i in range(0, len(temp_edges)):
                edges[group[i] - 1][0] += [temp_edges[i][0]]  # x array
                edges[group[i] - 1][1] += [temp_edges[i][1]]  # y array

        return edges

    def curved_lines(self, frame, edges, plot=False):
        line_frame = np.zeros_like(frame)
        h, w = frame.shape[:2]
        x_avg = -1
        phi_avg = -1
        lanes = []
        lane1 = [-1, -1, 0]
        lane2 = [-1, -1, 0]
        x_intercept1 = -1
        x_intercept2 = -1

        for idx in range(0, len(edges)):
            # fit quadratic equation to group of points to estimate lane curvature
            coeff = np.polyfit(edges[idx][1], edges[idx][0], 2)
            existing_lane = False
            # lane lines will have at least 50 points in worst case - anything shorter will be ignored
            if len(edges[idx][0]) > 75:
                if plot:
                    y = np.linspace(h/2, h, 5)
                    x = np.polyval(coeff, y)
                    points = (np.asarray([x, y]).T).astype(np.int32)
                    cv2.polylines(
                        line_frame, [points], False, (255, 255, 255), thickness=3)

            # find x intercept at "center" of rover
                x_line = coeff[0]*(4/3)*h**2 + coeff[1]*(4/3)*h + coeff[2]
            # find slope at 1/7th height up frame to anticipate curvature from first derivative of polynomial
                phi = (2*coeff[0]*(6*h/7) + coeff[1])

                for i in lanes:  # group lines with similar x intercepts together
                    if (x_line) - 100 <= i[0]/i[2] <= (x_line + 100):
                        i[0] += x_line
                        i[1] += phi
                        i[2] += 1
                        existing_lane = True
                        break

                if existing_lane == False:  # if the lane x_intercept isn't close to any other existing lane in lanes array
                    lanes += [[x_line, phi, 1]]
		    #print("Number of lanes: " + str(len(lanes)))
		    #print("Content of lanes: " + str(lanes))

                for idx, i in enumerate(lanes):
		    #print("Before if - idx :" + str(idx) + ", i: " + str(i))
                    if (i[2] > 1):  # lanes will have at least 2 lines combined from above, anything less than that can be ignored
			#print("Lanes length: " + str(len(lanes)))                        
			cv2.line(line_frame, (self.within_frame(
                            i[0]/i[2], w), h), (self.within_frame(i[0]/i[2], w), 9*h/10), (0, 255, 0), 3)
			#print("Green x intercept: " + str(i[0]/i[2]))
			#print("Green avg phi: " + str(i[1]/i[2]))

			#print("idx :" + str(idx) + ", i: " + str(i))
			#print("lane1 pre: " + str(lane1))
			#print("lane2 pre: " + str(lane2))

                        if i[2] > lane1[2]:
                            lane2 = lane1
                            lane1 = [i[0]/i[2], i[1]/i[2], i[2]]
                            x_intercept1 = i[0]/i[2]
			    #print("Lane 1: " + str(lane1))

                        elif i[2] > lane2[2] and (i[0]/i[2]) != lane1[0]:
                            lane2 = [i[0]/i[2], i[1]/i[2], i[2]]
                            x_intercept2 = i[0]/i[2]
			    #print("Lane 2: " + str(lane2))


			#print("lane1 post: " + str(lane1))
			#print("lane2 post: " + str(lane2))

                if lane1[0] != -1 and lane2[0] != -1:
                    
                    if not (x_intercept1 < WIDTH/2 and x_intercept2 < WIDTH/2):

                        # take average of right and left lines
                        x_avg = (lane1[0] + lane2[0]) / 2
		        #print("lane1 x: " + str(lane1[0]))
		        #print("lane2 x: " + str(lane2[0]))
		        #print("x_avg: " + str(x_avg))
                        # find angle in rads of slope
                        phi_avg = -1*np.arctan((lane1[1] + lane2[1]) / 2)*180/np.pi
                        cv2.line(line_frame, (self.within_frame(x_avg, w), h), (self.within_frame(
                            (x_avg - ((lane1[1] + lane2[1]) / 2)*h/5), w), 4*h/5), (0, 0, 255), 3)
                        self.last_lanes = [lane1[0], lane2[0]]

		if (x_avg - 320) != -321 and phi_avg != -1:
		    # Apply offsets
		    x_avg += X_OFFSET
		    phi_avg += PHI_OFFSET
		    cv2.putText(line_frame, ("x error :" + str(float((x_avg - 320)/320.0))), (50, 50),
		                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA, False)
        	    cv2.putText(line_frame, ("phi error :" + str(float(phi_avg/90.0))), (50, 100),
		                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA, False)
        return [line_frame, x_avg, phi_avg]

        #--Helper Function--#
    def within_frame(self, index, w):
        return int(min(w, max(0, index)))

    def show_vis(self, cv_masked, cv_final):
	
        # Draw mask on video
	cv2.line(cv_final, (int(TOP_LEFT_X), int(TOP_Y)), (int(WIDTH - TOP_RIGHT_X), int(TOP_Y)), (255, 0, 0), 2) #Top horizontal
	cv2.line(cv_final, (int(BOTTOM_LEFT_X), int(BOTTOM_Y)), (int(WIDTH - BOTTOM_RIGHT_X), int(BOTTOM_Y)), (255, 0, 0), 2) #Bottom horizontal
	cv2.line(cv_final, (int(TOP_LEFT_X), int(TOP_Y)), (int(BOTTOM_LEFT_X), int(BOTTOM_Y)), (255, 0, 0), 2) #Left vertical
	cv2.line(cv_final, (int(WIDTH - TOP_RIGHT_X), int(TOP_Y)), (int(WIDTH - BOTTOM_RIGHT_X), int(BOTTOM_Y)), (255, 0, 0), 2) #Right vertical

	# Draw mask on masked image
	cv2.line(cv_masked, (int(TOP_LEFT_X), int(TOP_Y)), (int(WIDTH - TOP_RIGHT_X), int(TOP_Y)), (255, 0, 0), 2) #Top horizontal
	cv2.line(cv_masked, (int(BOTTOM_LEFT_X), int(BOTTOM_Y)), (int(WIDTH - BOTTOM_RIGHT_X), int(BOTTOM_Y)), (255, 0, 0), 2) #Bottom horizontal
	cv2.line(cv_masked, (int(TOP_LEFT_X), int(TOP_Y)), (int(BOTTOM_LEFT_X), int(BOTTOM_Y)), (255, 0, 0), 2) #Left vertical
	cv2.line(cv_masked, (int(WIDTH - TOP_RIGHT_X), int(TOP_Y)), (int(WIDTH - BOTTOM_RIGHT_X), int(BOTTOM_Y)), (255, 0, 0), 2) #Right vertical
	
	# Display camera output
	cv2.namedWindow('cv_final_left')
	cv2.namedWindow('cv_masked_left')
	
	cv2.imshow('cv_final_left', cv_final)
	cv2.imshow('cv_masked_left', cv_masked)
	cv2.waitKey(1)
        return None


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