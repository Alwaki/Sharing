#!/usr/bin/env python2

'''

Short Description:      Program which attempts to detect simple geometric
                        shapes in a video feed.

Extended Description:   Subscripes to video topic, and analyzes each given
                        frame of video. This frame is altered to remove
                        noise and identify contours of images, and the
                        contour points are approximated by polynomial lines.
                        The number of lines determines the shape in the frame.
                        If no frame is given, then the program instead
                        defaults to no detection. The program structure is
                        set into four main modules, first user defined
                        functionality is given by the user (e.g. setting
                        parameters, camera) Second, libraries are imported,
                        and global declarations are made, etc.
                        Third, functions are defined (without prototyping).
                        Lastly, driver code is implemented.

Dependencies:           This program is intended to operate using ROS & Python.
                        Moreover, packages/libraries used are as follow:
                        - opencv
                        - rospy

Author(s):              Alexander Wallen Kiessling & Tim Gidlof

'''

##########################################################################
##                                USER SETUP
##########################################################################

'''
This section is intended to be changed by the user, to fit their
their requirements. However, some functionality does have default
values, which can be used if no changes are necessary.
'''

image_topic     = "raspicam_node/image"
detection_topic = "tag_detections_image"
image_encoding  = "bgr8"
area_min        = 200                # minimum area of contour for detection
area_max        = 8000              # maximum area of contour for detection
thresh_1        = 100
thresh_2        = 100
cnt_ret         = 'RETR_EXTERNAL'
cnt_met         = 'CHAIN_APPROX_NONE'
font            = 'FONT_HERSHEY_COMPLEX'


##########################################################################
##                                STATIC SETUP
##########################################################################

'''
This section should not be changed by users.
'''

# Import packages/libraries
import cv2
import rospkg
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##########################################################################
##                          FUNCTION DEFINITIONS
##########################################################################
'''
This section contains definitions. Descriptions of these are also
included, and a brief mention of important used variables (input & output)
'''

def getContours(image, output_image):

    '''

    Function Name:          getContours()

    Description:            This function uses an image which has recieved previous
                            treatment in the form of blur, grayscale, canny, and
                            dilation. It uses the findContours function from cv,
                            and detects contours in the image. These contours are
                            passed through a for loop, in which noise is removed
                            by verifying a legal size of the encompassed contour
                            area. By then connecting the contour points, we can see
                            if there are a number of lines which match a given
                            geometric shape. If there is a legal shape, this
                            information is written to the original image (without
                            blur, grayscale, etc) and given as output.

    Used Variables:         - image:        input image (modified to ease
                                            contour search)

                            - output_image: Original image which is returned
                                            with added information such as
                                            type, size, detection box.

    '''

    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if (area_min < area < area_max):

            arc = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * arc, True)
            lines = len(approx)
            type = ""

            flag = False

            if(lines==3):
                type = "triangle"
                flag = True
            elif(lines==4):
                flag = True
                type = "rectangle"
            elif(lines==5):
                type = "pentagon"
                flag = True

            if flag:
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(output_image, (x, y), (x+w, y+h), (0,0, 255), 5)
                cv2.putText(output_image, "Points: " + str(len(approx)), (x+w+20, y+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)
                cv2.putText(output_image, "Area: " + str(int(area)), (x+w+20, y+45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)
                cv2.putText(output_image, "Type:" + type, (x+w+20, y+70), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)

    return output_image


def imageManipulate(image):

    '''

    Function Name:          imageManipulate()

    Description:            This function takes an image, stores a copy of it
                            and passes the image several cv methods which makes
                            it easier to find contours in the image.
                            The contours are found with the help of getContours().

    Used Variables:         - image:        input image with cv format.

                            - output_image: returned value, original image with
                                            added information regarding shape
                                            detection.


    '''

    output_image =  image.copy()
    blurred_image = cv2.GaussianBlur(image, (7, 7), 1)
    gray_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
    canny_image = cv2.Canny(gray_image, thresh_1, thresh_2)
    kernel = np.ones((5,5))
    dilated_image = cv2.dilate(canny_image, kernel, iterations = 1)
    output_image = getContours(dilated_image, output_image)
    return output_image

class detector:

    '''
    Class Name:             imageManipulate()

    Description:            This function takes an image, stores a copy of it
                            and passes the image several cv methods which makes
                            it easier to find contours in the image.
                            The contours are found with the help of getContours().

    Attributes:             - bridge:

                            - image_sub:

                            - image_pub:

    Methods:                - callback():


    '''

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=2)
        self.image_pub = rospy.Publisher(detection_topic, Image, queue_size=2)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, image_encoding)
        output_image = imageManipulate(cv_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, image_encoding))


def main():

    '''

    Function Name:          main()

    Description:            The main function of the program. Initializes the
                            node as per ROS default, creates the detection
                            instance, and hands over control to roscore.

    Used Variables:         n/a

    '''

    rospy.init_node("shape_detector.py", anonymous = True)
    image_converter = detector()
    rospy.spin()

##########################################################################
##                          DRIVER CODE
##########################################################################

'''
This section is intended to initialize any code which requires startup prompt
to be used in applications.
'''

if __name__ == '__main__':
    main()
