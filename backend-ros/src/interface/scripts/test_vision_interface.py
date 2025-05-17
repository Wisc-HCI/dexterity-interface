#!/usr/bin/env python

"""
Test objects that mimic what the vision system returns by 
randomly moving 3 objects around.
"""

import rospy
import copy
import random 

from std_msgs.msg import Header
from interface.msg import Object, ObjectArray



class VisionInterface:
    def __init__(self):
        rospy.init_node('vision_interface')

        self.objects = [ {'label': 'apple'}, {'label': 'bread'}, {'label': 'peanut_butter'} ]
        self.generate_random_coords(None)

        self.object_pub = rospy.Publisher("/scene/vision/objects", ObjectArray, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.object_monitor)
        rospy.Timer(rospy.Duration(30), self.generate_random_coords) # Adjust object positions every so often

        rospy.spin()


    def generate_random_coords(self, event):
        for obj in self.objects:
            obj['x'] = random.uniform(0.3, 0.5)
            obj['y'] = random.uniform(-0.15, 0.15) 


    def object_monitor(self, event):
        dim = 0.05

        msg = ObjectArray()
        for each_obj in self.objects:
            obj = Object()
            obj.header = Header()
            obj.header.stamp = rospy.Time.now()
            obj.id = each_obj['label']
            obj.description = each_obj['label']
            obj.x = each_obj['x']
            obj.y = each_obj['y']
            obj.z = 0.025  
            obj.width = dim
            obj.length = dim
            obj.height = dim

            msg.objects.append(obj)

        self.object_pub.publish(msg)

            


if __name__=="__main__":
    scene_handler = VisionInterface()
