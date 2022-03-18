#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

    def __init__(self):
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 2nd Lab Session
        self.velocity_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist)

        # Initialise any flags that signal a colour has been detected (default to false)
        self.detected_green = 0
        self.detected_red = 0

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.desired_velocity = Twist()
        self.desired_velocity.linear.x = 0.0 #linear velocity

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.cv_bridge = CvBridge()
        self.image_publisher = rospy.Publisher('camera/rgb/image_raw',Image)
        self.image_subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)


    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
         cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
         image_copy = cv_image.copy()
        except CvBridgeError as e:
            print(e)

        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179

        #green
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        #red
        hsv_red_lower = np.array([0, 100, 100])
        hsv_red_upper = np.array([20, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        mask_red  = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        mask_color_green = cv2.bitwise_and(image_copy, image_copy, mask = mask_green)
        mask_color_red = cv2.bitwise_and(image_copy, image_copy, mask = mask_red)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        mask_green_to_gray = cv2.cvtColor(mask_color_green, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(mask_green_to_gray, 50, 255, 0)
        contours, heirarchy = cv2.findContours (thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        mask_red_to_gray = cv2.cvtColor(mask_color_red, cv2.COLOR_BGR2GRAY)
        ret2,thresh2 = cv2.threshold(mask_red_to_gray, 50, 255, 0)
        contours2, heirarchy2 = cv2.findContours (thresh2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Loop over the contours
        if len(contours) > 0:

            # Loop over the contours
            # There are a few different methods for identifying which contour is the biggest:
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour

            self.green = max(contours, key = cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(self.green) > 7000: #<What do you think is a suitable area?>:
                print ("Green detected and circled in blue")
                # Alter the value of the flag
                self.detected_green = 1

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                (w, h), radius = cv2.minEnclosingCircle(self.green)
                center = (int(w),int(h))
                radius = int(radius)
                cv2.circle(mask_color_green,center,radius,(255, 0, 0),2)

        if len(contours2) > 0:

            # Loop over the contours
            # There are a few different methods for identifying which contour is the biggest:
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour

            self.red = max(contours2, key = cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(self.red) > 3000: #<What do you think is a suitable area?>:
                print ("Red detected and circled in blue")
                # Alter the value of the flag
                self.detected_red = 1

                (w, h), radius = cv2.minEnclosingCircle(self.red)
                center = (int(w),int(h))
                radius = int(radius)
                cv2.circle(mask_color_red,center,radius,(255, 0, 0),2)

        #Check if a flag has been set = green object detected - follow the colour object

        if self.detected_green == 1 and self.detected_red == 0 and cv2.contourArea(self.green) > 3000 :

            print ("Moving towards green")
            #desired_velocity = Twist()
            #desired_velocity.linear.x = 0.5 #linear velocity
            #rate = rospy.Rate(10)
            #self.velocity_publisher.publish(desired_velocity)
            #rate.sleep()

        # Be sure to do this for the other colour as well
        #Setting the flag to detect red, and stop the turtlebot from moving if red is detected
        elif self.detectedGreen == 1 and self.detectedRed == 1:
                    # stop if red is detected
                    print ("Red detected, stopping")
                    #desired_velocity = Twist()
                    #desired_velocity.linear.x = 0.0 #linear velocity
                    #rate = rospy.Rate(10)
                    #self.velocity_publisher.publish(desired_velocity)
                    #rate.sleep()


    #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow('camera_Feed', cv_image)
        cv2.imshow('mask1', mask_color_green)
        cv2.imshow('mask2', mask_color_red)
        cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cI = colourIdentifier()
    rospy.init_node('image_converter', anonymous=True)
    rate = rospy.Rate(10)

    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap it in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
