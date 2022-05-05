#!/usr/bin/env python

import yaml
import rospy
from movement import Move
import numpy as np

if __name__ == '__main__':

    GreenFlag = False
    CluedoFlag = False
    room = -1
    try:
        rospy.init_node('navigation', anonymous = True)
        navigator = Move()
         


        CONST_POINTS_PATH = '.. /world/input_points.yaml'

        from yaml.loader import SafeLoader #import safeloader from pyyaml

        with open(CONST_POINTS_PATH) as f:
            data = yaml.load(f, Loader=SafeLoader)
    
    
        keys = data.keys()
        for key in range(len(keys)):
            coord = data[keys[key]]

            if key == 1 and not GreenFlag:
                continue
            if GreenFlag and CluedoFlag:
                break  

            
            x = coord[0]
            y = coord[1]
            theta = 0
            coordinate = {'x': x, 'y': y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}     

            rospy.loginfo("Go to (%s, %s)", coordinate['x'], coordinate['y'])
            success = navigator.move(coordinate, quaternion)

            if success:
                rospy.loginfo("Reached Coordinates")
            else:
                rospy.loginfo("Failed to Reach Coordinates")

            rospy.sleep(1)

            if key == 0: # switch between 0 and 2 for different rooms
                GreenFlag = True

            if key == 1: # switch between 1 and 3 for differnet rooms
                CluedoFlag = True
            
    except rospy.ROSInterruptException:
        rospy.logino("Ctrl-C Found - Quitting")

