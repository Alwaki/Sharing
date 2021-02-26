#!/usr/bin/env python2

'''

Short Description:      Program which attempts to detect simple geometric
                        shapes in a video feed, and publish detections.

Extended Description:   Subscripes to video topic, and analyzes each given
                        frame of video. This frame is altered to remove
                        noise and identify contours of images, and the
                        contour points are approximated.
                        The number of lines determines the shape in the frame.
                        The program structure is set into four main modules,
                        first user defined functionality is given by the user
                        (e.g. setting parameters, camera).
                        Second, libraries are imported, and global declarations
                        are made, etc. Third, functions are defined
                        (without prototyping). Lastly, driver code is implemented.

Dependencies:           This program is intended to operate using ROS & Python.
                        Moreover, packages/libraries used are as follow:
                        - opencv
                        - rospy
                        - numpy
                        - sensor_msgs
                        - cv_bridge

Author(s):              Alexander Wallen Kiessling & Tim Gidlof

'''

##########################################################################
##                                USER SETUP
##########################################################################

'''
This section is intended to be changed by the user, to fit their
requirements. However, some functionality does have default values,
which can be used if no changes are necessary.
'''

image_topic     = "raspicam_node/image"
detection_topic = "tag_detections_image"
image_encoding  = "bgr8"
node_name       = "shape_detector.py"
anon_flag       = True                  # boolean for anonymous publishing in ROS
area_min        = 150                   # minimum area of contour for detection
area_max        = 1100                  # maximum area of contour for detection
thresh_low      = 10                    # low threshhold of canny edge detection
thresh_high     = 200                   # high threshhold of canny edge detection
queue_sub       = 2                     # numbers of images stored for subscription
queue_pub       = 2                     # numbers of images stored for publishing
blur_kernel     = (7, 7)                # kernel size for blur
blur_std        = 1                     # standard deviation (x and y) for blur
dilation_iter   = 1                     # dilation iterations
y_lb            = 160                   # lower bound of height of image analysis
y_ub            = 320                   # upper bound of height of image analysis
closed_flag     = True                  # boolean for if contour figures are closed
color_lb        = [28, 44, 12]          # lower color mask boundary (BGR)
color_ub        = [100, 109, 40]        # upper color mask boundary (BGR)
 
##########################################################################
##                                STATIC SETUP
##########################################################################

'''
This section should not be changed by users.
'''

# Import packages/libraries
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##########################################################################
##                          FUNCTION DEFINITIONS
##########################################################################

'''
This section contains all function definitions, as well as leading descriptions
on how the classes/functions operate.
'''

class ShapeDetector:

    '''

    Class Name:             ShapeDetector

    Class Description:      This class contains attributes that start a
                            subscription and publishing service. It
                            subscribes to a camera feed to recieve images,
                            and uses its methods to detect any geometric
                            shapes within the frame of the image. To detect
                            the shapes, it uses contours, which are found
                            and counted to correlate to different geometric
                            shapes. Detection boxes are drawn on the
                            original image, and published to a detection
                            topic.

    '''

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size = queue_sub)
        self.image_pub = rospy.Publisher(detection_topic, Image, queue_size = queue_pub)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, image_encoding)
        copy_image = cv_image.copy()
        dilated_image = self.imageManipulate(cv_image)
        contours = self.findContours(dilated_image)
        output_image = self.findShape(contours, copy_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, image_encoding))

    def imageManipulate(self, image):
        color_lower = np.array(color_lb, np.uint8) 
        color_upper = np.array(color_ub, np.uint8) 
        mask = cv2.inRange(image, color_lower, color_upper)
        masked_image = cv2.bitwise_and(image, image, mask = mask)
        blurred_image = cv2.GaussianBlur(masked_image, blur_kernel, blur_std)
        gray_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
        canny_image = cv2.Canny(gray_image, thresh_low, thresh_high)
        dilated_image = cv2.dilate(canny_image, np.ones(3), iterations = dilation_iter)
        return dilated_image

    def findContours(self, image):
        contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def findShape(self, contours, output_image):
        for contour in contours:
            area = cv2.contourArea(contour)
            if area_min < area < area_max:
                arc = cv2.arcLength(contour, closed_flag)
                approx = cv2.approxPolyDP(contour, 0.05 * arc, closed_flag)
                x, y, w, h = cv2.boundingRect(approx)
                if y_lb < y < y_ub:
                    lines = len(approx)
                    type = ""
                    flag = False
                    if lines == 3:
                        if 0.9 <= w / float(h) <= 1.1:
                            type = "triangle"
                            flag = True
                    elif lines == 4:
                        if 0.95 <= w / float(h) <= 1.05:
                            type = "square"
                            flag = True
                    elif lines == 5:
                        type = "pentagon"
                        flag = True
                    if flag:
                        cv2.rectangle(output_image, (x, y), (x+w, y+h), (0,0, 255), 5)
                        cv2.putText(output_image, "Size: " + str(int(area)), (x+w+20, y+45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (50,200,50), 2)
                        cv2.putText(output_image, "Type:" + type, (x+w+20, y+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (50,200,50), 2)
        return output_image


##########################################################################
##                          ROS NODE START
##########################################################################

'''
This section is intended to initialize the node within the ROS framework,
listing the node with roscore, initializing the class instance and handing
over control to the core.
'''

if __name__ == '__main__':
    rospy.init_node(node_name, anonymous = anon_flag)
    sd = ShapeDetector()
    rospy.spin()
