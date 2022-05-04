#!/usr/bin/env python

import yaml
import math
import rospy
from movement import Move
import numpy as np
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from identify_circles import colourIdentifier
from img_cla_dect import characterIdentifier
import time


global bump
bump = False

def spin():
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
    #rospy.init_node('Walker', anonymous=True)
    rate = rospy.Rate(10)
    desired_velocity = Twist()
    desired_velocity.angular.z = 0.6
    pub.publish(desired_velocity)

def processBump(data):
        global bump
        if (data.state == BumperEvent.PRESSED ):
                bump = True
        else:
                bump = False

def listen():
    sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, processBump)

def rotate(coord,n, thetas):
    theta = thetas
    temp = theta + (math.pi*2)
        


    x = coord[0]
    y = coord[1]
    coordinate = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)} 
    print("straight")
    navigator.move(coordinate, quaternion)


    for i in range(n):
        print(i)
        if i < n/2:
            theta = theta + (math.pi*2)/(2*n)
        else:
            temp = temp - (math.pi*2)/(2*n)
            theta = temp

        print("theta = ", theta)
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)} 
        navigator.move(coordinate, quaternion)

        rospy.sleep(0.5)
        test = characterIdentifier()
        rospy.sleep(1)
        if test.found != 0:
            return 1
        #theta = theta + (math.pi*2)/n

def search(coord, thetas):
    theta = thetas
    x = coord[0]
    y = coord[1]
    coordinate = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)} 
    #quaternion = {'r1' : 0, 'r2' : 0, 'r3' : 1, 'r4' : theta } #np.cos(theta)+np.sin(theta)*1
    navigator.move(coordinate, quaternion)


    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
    #rospy.init_node('Walker', anonymous=True)
    rate = rospy.Rate(10)
    desired_velocity = Twist()
    desired_velocity.linear.x = 0.5
    while bump == False:
        pub.publish(desired_velocity)
        listen()
    desired_velocity.linear.x = -0.6
    for i in range(15):
        pub.publish(desired_velocity)
        rate.sleep()
    listen()

    

    #rospy.sleep(1)

    theta = theta + (math.pi*2)/8

    return theta

if __name__ == '__main__':


    GreenFlag = False
    CluedoFlag = False
    room = -1
    try:
        rospy.init_node('navigation', anonymous = True)
        navigator = Move()
         
        CONST_POINTS_PATH = '..//world//input_points.yaml'

        from yaml.loader import SafeLoader #import safeloader from pyyaml

        with open(CONST_POINTS_PATH) as f:
            data = yaml.load(f, Loader=SafeLoader)
    

        keys = data.keys()
        
        coord = 0
        for key in range(len(keys)):

            if key == 1 and not GreenFlag:
                continue
            if GreenFlag and CluedoFlag:
                break  

            coord = data[keys[key]]
            x = coord[0]
            y = coord[1]
            centerCoord = coord
            theta = 0
            coordinate = {'x': x, 'y': y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}     

            rospy.loginfo("Go to (%s, %s)", coordinate['x'], coordinate['y'])
            success = navigator.move(coordinate, quaternion)

            if success:
                rospy.loginfo("Reached Coordinates")
            else:
                rospy.loginfo("Failed to Reach Coordinates")

            rospy.sleep(0.1)

            

            if key == 0:
                GreenFlag = True

            if key == 1: # switch between 1 and 3 for differnet rooms
                CluedoFlag = True
        

       # quaternion = {'r1' : 0, 'r2' : 0, 'r3' : 1, 'r4' : 1 } #np.cos(theta)+np.sin(theta)*1
        #navigator.move(coordinate, quaternion)

        #rospy.sleep(3)

        #quaternion = {'r1' : 0, 'r2' : 0, 'r3' : 1, 'r4' : 1.195 } #np.cos(theta)+np.sin(theta)*1
        #navigator.move(coordinate, quaternion)
        

        x = 0.0
        print(math.pi)
        for i in range(9):
            if i == 8:
                break
            print("search")
            temp = x
            x = search(centerCoord, x)
            navigator.currentPos()
            coord = [navigator.posx, navigator.posy]
            print("rotate")
            t = rotate(coord, 4, temp)
            print("return")
            if t == 1:
                print("HERE BITCH")
                break
            navigator.move(coordinate, quaternion)
    

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Found - Quitting")

