#!/usr/bin/env python

"""
Handles the objects in the scene.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
import tf
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Header
from interface.msg import Object, ObjectArray



class SceneHandler:
    def __init__(self):
        rospy.init_node('scene_handler')
    
        self.server = InteractiveMarkerServer("scene/object_controls")
        self.br = TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/scene/vision/objects", ObjectArray, self.vision_sub)
        self.object_pub = rospy.Publisher("/scene/objects", ObjectArray, queue_size=10)

        self.objects = {}  # Dictionary of current objects in scene
    

        # Timer to publish transforms and monitor objects in scene
        rospy.Timer(rospy.Duration(0.05), self.frame_callback)
    

        rospy.spin()


    def frame_callback(self, event):
        
        # Use a list of keys to avoid modifying the dictionary during iteration
        time = rospy.Time.now()
        if self.objects:
            for obj_id in list(self.objects.keys()):
                obj = self.objects[obj_id]
                self.br.sendTransform( (obj.x, obj.y, obj.z), (0, 0, 0, 1.0), time, obj.frame_id, "scene")
            
        # If no objects, broadcast temporary object so scene is broadcast
        else:
            self.br.sendTransform( (0,0,0), (0, 0, 0, 1.0), time, "object/none", "scene")


    def process_feedback(self, feedback ):
        s = "Feedback from object '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo( s + ": button click" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed")

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )
        self.server.applyChanges()


    
    def add_object(self, vision_obj: Object) -> str:
        """
        vision_obj.id: MUST BE UNIQUE per object
        """
        frame_id = f"object/{vision_obj.id}"
        vision_obj.frame_id = frame_id

        
        # If object already exists, don't creat it again
        if vision_obj.id in self.objects: 
            self.objects[vision_obj.id] = vision_obj  # Add to object list
            return

        # Else, create the object
        self.objects[vision_obj.id] = vision_obj


        # Wait for the transform to become available
        try:
            self.tf_listener.waitForTransform("scene", vision_obj.frame_id, rospy.Time(0), rospy.Duration(2.0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Timeout or error waiting for TF frame {vision_obj.frame_id}: {e}")


        object = InteractiveMarker()
        object.header.frame_id = frame_id 
        object.pose.position = Point(0,0,0)
        object.scale = 1
        object.name = vision_obj.id 

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        object.controls.append(copy.deepcopy(control))

        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = vision_obj.length
        marker.scale.y = vision_obj.width
        marker.scale.z = vision_obj.height
        marker.color.r = 1
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        # Add text marker for description label
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = vision_obj.description
        text_marker.scale.x = vision_obj.width
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.pose.position.z = vision_obj.z + vision_obj.height/2 + 0.05 # A bit above the object

        control.markers.append( marker )
        control.markers.append(text_marker)

        control.always_visible = True
        object.controls.append(control)


   
        self.server.insert(object, self.process_feedback)
        
        self.server.applyChanges()


    
        
    

    def vision_sub(self, msg):

        # Add or update objects detected by vision model to scene
        for obj in msg.objects:
            self.add_object(obj)

        # Publish updated scene
        msg_out = ObjectArray()
        for _, obj in self.objects.items():
            msg_out.objects.append(obj)

        self.object_pub.publish(msg_out)
        

        


if __name__=="__main__":
    scene_handler = SceneHandler()
