#!/usr/bin/env python

"""
Vision system that looks for objects
"""

from assistive_robotics_thesis.src.florence_2_L.main import warmup, command
from interface.msg import Object, ObjectArray

import rospy
import copy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
import random



class VisionInterface:
    def __init__(self):
        rospy.init_node('vision_interface')

        self.model, self.processor = warmup(show_images=True)

        self.tf_listener = tf.TransformListener()
    
        self.object_pub = rospy.Publisher("/scene/vision/objects", ObjectArray, queue_size=10)

        rospy.Timer(rospy.Duration(1), self.object_monitor)
        print("STARTING DETECTION NOW.")

        rospy.spin()


    def mm_to_M(self, mm:float) -> float:
        """
        Convert millimeters to meters
        """
        M = float(mm) / 1000
        return M
    

    def camera_to_scene_transform(self, x, y, z):
        """Convert  camera feed to scene frame of reference"""

        source = "camera_base_rgb"
        goal = "scene"

        point = PointStamped()
        point.header.frame_id = source
        point.header.stamp = rospy.Time(0)
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.tf_listener.waitForTransform(goal, source, rospy.Time(0), rospy.Duration(4.0))
        point_in_scene = self.tf_listener.transformPoint(goal, point)

        return (point_in_scene.point.x, point_in_scene.point.y, point_in_scene.point.z)


    def object_monitor(self, event):

        objects = command(self.model, self.processor, "")

        msg = ObjectArray()
        for objDict in objects:
            
            # TODO: Make different width, len, height
            # dim = self.mm_to_M(objDict['depth_max']) - self.mm_to_M(objDict['depth_min'])
            dim = 0.05

            try:
                x, y, z = self.camera_to_scene_transform( 
                    self.mm_to_M(objDict['center_coord'][0]), 
                    self.mm_to_M(objDict['center_coord'][1]),
                    self.mm_to_M(objDict['center_coord'][2])
                )
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(f"TF transform failed: {e}")
                continue
                
            obj = Object()
            obj.header = Header()
            obj.header.stamp = rospy.Time.now()
            obj.id = objDict['label']
            obj.description = objDict['label']
            obj.x = x
            obj.y = y
            obj.z = z
            obj.width = dim
            obj.length = dim
            obj.height = dim

            msg.objects.append(obj)


        

        self.object_pub.publish(msg)

        


if __name__=="__main__":
    scene_handler = VisionInterface()
