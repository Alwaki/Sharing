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
area_min        = 20                # minimum area of contour for detection
area_max        = 8000              # maximum area of contour for detection
thresh_1        = 25
thresh_2        = 25
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

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


##########################################################################
##                          FUNCTION DEFINITIONS
##########################################################################
'''
This section contains function definitions. Descriptions of these are also
included, and a brief mention of important used variables (input & output)
'''

'''
Function Name:          getContours()

Description:

Used Variables:         - image:
                        - contoured_image:

'''
def getContours(image, output_image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if (area_min < area < area_max):

            arc = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * arc, True)
            lines = len(approx)
            type = ""

            if(lines==3):
                type = "triangle"
                print("triangle!")
            elif(lines==4):
                type = "rectangle"
                print("recntangle!")
            elif(lines==5):
                type = "pentagon"
                print("pentagon!")
            else:
                type = "unidentified"
                print("nothing!")

            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(output_image, (x, y), (x+w, y+h), (0,255, 0), 5)
            cv2.putText(output_image, "Points: " + str(len(approx)), (x+w+20, y+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(output_image, "Area: " + str(int(area)), (x+w+20, y+45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(output_image, "Type:" + type, (x+w+20, y+70), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)
    return output_image

def image_manipulate(image):
        output_image =  image.copy()
        blurred_image = cv2.GaussianBlur(image, (7, 7), 1)
        gray_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
        canny_image = cv2.Canny(gray_image, thresh_1, thresh_2)
        kernel = np.ones((5,5))
        dilated_image = cv2.dilate(canny_image, kernel, iterations = 1)
        output_image = Contours(dilated_image, output_image)
        return output_image
'''
class
'''

class detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=2)
        self.image_pub = rospy.Publisher(detection_topic, Image, queue_size=2)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, image_encoding)
            output_image = image_manipulate(cv_image)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, image_encoding))
        except:
            print("Error callback")

def main():
    print("HIWAPFOAWIOHALIWHFAWGF")
    image_converter = detector()
    rospy.init_node("shape_detector.py", anonymous = True)
    print("HIWAPFOWUBVADUIYGVWDUYAWGV")
    rospy.spin()

##########################################################################
##                          DRIVER CODE
##########################################################################
if __name__ == '__main__':
    main()
