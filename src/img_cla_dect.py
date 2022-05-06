#!/usr/bin/env python

# Detecting poster colour to determine character
from __future__ import division
import imp
from pydoc import classname
import cv2
import numpy as np
import os
import sys
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from os.path import join

# CHANGE HARDCODED PATH
path = '../src/imgs_train'
# Initiate ORB detector
orb = cv2.ORB(nfeatures=2000)

#### Import Images
images = []
classNames = []
myList = os.listdir(path)


for cl in myList:
    imgCur = cv2.imread(join(path,cl),0)
    images.append(imgCur)
    classNames.append(os.path.splitext(cl)[0])


def findDes(images):
    desList = []
    for img in images:
        kp = orb.detect(img,None)
        kp, des = orb.compute(img,kp)
        desList.append(des)
    return desList

desList = findDes(images)


def findID(img, deList):
    kp2 = orb.detect(img,None)
    kp2, des2 = orb.compute(img,kp2)
    bf = cv2.BFMatcher()
    matchList = []
    finalVal = -1
    try:
        for des in desList:
            matches = bf.knnMatch(des,des2,k=2)
            good = []
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    good.append([m])

            matchList.append(len(good))
    except:
        pass
    # print(matchList)

    
    if len(matchList)!=0:
        if (max(matchList) > 30) :
            finalVal = matchList.index(max(matchList))
            print("-++++_")
    return finalVal

# s = 0
# m = 0
# p = 0
# pl = 0
# f = 0
global cv_image

class characterIdentifier():

    def __init__(self):
        self.sensitivity = 5
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        # self.scarlett_flag = 0
        # self.peacock_flag = 0
        # self.mustard_flag = 0
        # self.plum_flag = 0
        self.found = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            
        except:
            print("Conversion failed!")

        

    # def colourDetect(self):
    #     print("checking colour") 

              

        # Upper and lower bounds for the four colours to identify
        hsv_scarlett_lower = np.array([0-self.sensitivity,100,10])
        hsv_scarlett_upper = np.array([0+self.sensitivity,255,255])
        hsv_peacock_lower = np.array([105-self.sensitivity,100,10])
        hsv_peacock_upper = np.array([105+self.sensitivity,255,255]) # can't see her dress
        hsv_mustard_lower = np.array([27-self.sensitivity,100,10])
        hsv_mustard_upper = np.array([27+self.sensitivity,255,255])
        hsv_plum_lower = np.array([153-self.sensitivity,100,10])
        hsv_plum_upper = np.array([153+self.sensitivity,225,255]) # not as clear as others
        # Convert the bgr image into a hsv image
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        id = findID(img,desList)
        # Filter out everything but particular colours using the cv2.inRange() method
        mask1 = cv2.inRange(hsv_image,hsv_scarlett_lower,hsv_scarlett_upper)
        mask2 = cv2.inRange(hsv_image,hsv_peacock_lower,hsv_peacock_upper)
        mask3 = cv2.inRange(hsv_image,hsv_mustard_lower,hsv_mustard_upper)
        mask4 = cv2.inRange(hsv_image,hsv_plum_lower,hsv_plum_upper)

        # global s, m, p, pl, f

        # # detecting plum
        # img_plum = cv2.bitwise_and(cv_image,cv_image,mask=mask4)
        # contours_plum, hierarchy = cv2.findContours(mask4, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # if len(contours_plum) > 0:
        #     c = max(contours_plum,key=cv2.contourArea)
        #     if cv2.contourArea(c) > 500:
        #         x, y, w, h = cv2.boundingRect(c)
        #         cv2.rectangle(img_plum,(x,y),(x+w, y+h),(0,0,255),2)
        #         self.plum_flag = 1
        #         pl = 1
        #         print("found plum colour")
                
                
        # # detecting peacock
        # img_peacock = cv2.bitwise_and(cv_image,cv_image,mask=mask2)
        # contours_peacock, hierarchy = cv2.findContours(mask2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # if len(contours_peacock) > 0:
        #     c = max(contours_peacock,key=cv2.contourArea)
        #     if cv2.contourArea(c) > 1000:
        #         x, y, w, h = cv2.boundingRect(c)
        #         cv2.rectangle(img_peacock,(x,y),(x+w, y+h),(0,0,255),2)
        #         self.peacock_flag =1
        #         p = 1
        #         print("found peacock colour")

        # # detecting scarlett
        # img_scarlett = cv2.bitwise_and(cv_image,cv_image,mask=mask1)
        # contours_scarlett, hierarchy = cv2.findContours(mask1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # if len(contours_scarlett) > 0:
        #     c = max(contours_scarlett,key=cv2.contourArea)
        #     if cv2.contourArea(c) > 1000:
        #         x, y, w, h = cv2.boundingRect(c)
        #         cv2.rectangle(img_scarlett,(x,y),(x+w, y+h),(0,0,255),2)
        #         self.scarlett_flag = 1
        #         s = 1
        #         print("found scarlett colour")

        # # detecting mustard
        # img_mustard = cv2.bitwise_and(cv_image,cv_image,mask=mask3)
        # contours_mustard, hierarchy = cv2.findContours(mask3, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # if len(contours_mustard) > 0:
        #     c = max(contours_mustard,key=cv2.contourArea)
        #     if cv2.contourArea(c) > 1000:
        #         x, y, w, h = cv2.boundingRect(c)
        #         cv2.rectangle(img_mustard,(x,y),(x+w, y+h),(0,0,255),2)
        #         self.mustard_flag = 1
        #         m = 1
        #         print("found mustard colour")
               
        # self.scarlett_flag = s
        # self.peacock_flag = p
        # self.mustard_flag = m
        # self.plum_flag = pl

        ########################################################################################################################################
        # THE WORKING CODE:
        if( findID(img,desList) != 0 ):
            # detecting plum
            
            img_plum = cv2.bitwise_and(cv_image,cv_image,mask=mask4)
            contours_plum, hierarchy = cv2.findContours(mask4, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_plum) > 0:
                c = max(contours_plum,key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(img_plum,(x,y),(x+w, y+h),(0,0,255),2)
                    
                    print("found plum")
                 
                    cv2.imwrite('../output/cluedo_character.png', cv_image)
                    self.found = 4
                    
                    file = open("../output/cluedo_character.txt", "wb+")
                    file.write("Professor Plum") 
                    file.close()
                    sys.exit()
            # detecting peacock
            img_peacock = cv2.bitwise_and(cv_image,cv_image,mask=mask2)
            contours_peacock, hierarchy = cv2.findContours(mask2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_peacock) > 0:
                c = max(contours_peacock,key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(img_peacock,(x,y),(x+w, y+h),(0,0,255),2)
                    
                    print("found peacock colour")

                    cv2.imwrite('../output/cluedo_character.png', cv_image)
                    self.found = 3
                    
                    print("Peacock found!")
                    file = open("../output/cluedo_character.txt", "wb+")
                    file.write("Mrs Peacock")
                    file.close()
                    sys.exit()


            # detecting scarlett
            img_scarlett = cv2.bitwise_and(cv_image,cv_image,mask=mask1)
            contours_scarlett, hierarchy = cv2.findContours(mask1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_scarlett) > 0:
                c = max(contours_scarlett,key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(img_scarlett,(x,y),(x+w, y+h),(0,0,255),2)
                    
                    print("found scarlett")

                    cv2.imwrite('../output/cluedo_character.png', cv_image)

                    file = open("../output/cluedo_character.txt", "wb+")
                    file.write("Miss Scarlett")
                    file.close()
                    self.found =  1
                    
                    sys.exit()


            # detecting mustard
            img_mustard = cv2.bitwise_and(cv_image,cv_image,mask=mask3)
            contours_mustard, hierarchy = cv2.findContours(mask3, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_mustard) > 0:
                c = max(contours_mustard,key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(img_mustard,(x,y),(x+w, y+h),(0,0,255),2)
                
                    print("found mustard colour")

                    cv2.imwrite('../output/cluedo_character.png', cv_image)
                    self.found = 2
                    print("Mustard found!")
                    file = open("../output/cluedo_character.txt", "wb+")
                    file.write("Colonel Mustard")
                    file.close()
                    sys.exit()
        
        #################################################################################################################################################
        
        # if( findID(img,desList) == 3 ) and (self.plum_flag==1):
        #     print("****************")
        #     cv2.imwrite('cluedo_character.png', cv_image)
        #     self.found = 4
        #     f = 4
        #     print("Plum found!")
        #     file.write("Professor Plum") 
        #     file.close()
        #     sys.exit()

        # if self.plum_flag == 1:
        #     pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        #     rate = rospy.Rate(10)
        #     desired_velocity = Twist()
        #     desired_velocity.angular.z = 0
        #     desired_velocity.linear.x = 0.2
        #     pub.publish(desired_velocity)
        #     if (id == 1) and (self.plum_flag==1):
        #         desired_velocity.linear.x = 0
        #         pub.publish(desired_velocity)
        #         print("yo")

        # file = open("cluedo_character.txt", "wb+")

        

        # if (id == 0) and (self.scarlett_flag==1):
        #     #cv2.putText(cv_image,classNames[id],(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        #     print("Scarlett found!")
        #     while True:
        #         file.write("Miss Scarlett")
        #         cv2.imwrite('cluedo_character.png', cv_image)
        #         file.close()
        #         self.found =  1
        #         f = 1
        #         sys.exit()
            
        # elif (id == 1) and (self.mustard_flag==1):
        #     #cv2.putText(cv_image,classNames[id],(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        #     cv2.imwrite('cluedo_character.png', cv_image)
        #     self.found = 2
        #     f = 2
        #     print("Mustard found!")
        #     file.write("Colonel Mustard")

        # elif (id == 2) and (self.peacock_flag==1):
        #     #cv2.putText(cv_image,classNames[id],(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        #     cv2.imwrite('cluedo_character.png', cv_image)
        #     self.found = 3
        #     f = 3
        #     print("Peacock found!")
        #     file.write("Mrs Peacock")

        # elif (id == 3) and (self.plum_flag==1):
        #     #cv2.putText(cv_image,classNames[id],(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        #     cv2.imwrite('cluedo_character.png', cv_image)
        #     self.found = 4
        #     f = 4
        #     print("Plum found!")
        #     file.write("Professor Plum") 
        #     file.close()
        #     sys.exit()

        # self.found = f

        # # cv2.namedWindow('camera_Feed')
        # # cv2.imshow('camera_Feed', cv_image)
        # # cv2.waitKey(0)  

def main(args):
    rospy.init_node('Camera', anonymous=True)
    cI = characterIdentifier()
    rospy.sleep(1)
    print(cI.found)
    try:
        rospy.spin()
    except KeyboardInterrupts:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
