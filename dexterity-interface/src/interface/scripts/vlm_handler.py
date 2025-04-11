#!/usr/bin/env python
import rospy, os
from llm_interface import LLMInterface
from std_msgs.msg import String
from interface.msg import Object, ObjectArray

class VLMHandler():
    def __init__(self):
        rospy.init_node('vlm_handler')

        self.current_objects = {}
        rospy.Subscriber("/scene/objects", ObjectArray, self.object_sub)

        vlm_role = ("You are a helpful assistant that supports a vision-language system by" + 
                    "interpreting natural language instructions and refining object detection " +
                    "results. Your responsibilities include: (1) Extracting distinct object labels " +
                    "from user input, even when they are not clearly separated by commas or " +
                    "conjunctions (e.g., 'I want the water bottle and the tissues' should result in " +
                    "['water bottle', 'tissues']); (2) Comparing these desired labels against a list " +
                    "of currently detected objects; (3) Clearly identifying which objects were " +
                    "correctly found, which were missing, and which were detected but not desired; " +
                    "and (4) If necessary, rewriting a clean, improved grounding prompt that a " +
                    "vision-language model (specifically Florence-2) can use to detect only the " +
                    "intended objects. Always ensure that the extracted labels are specific and " +
                    "correctly segmented, regardless of how the user phrases their request.")

        self.llm = LLMInterface(vlm_role)

        example_query = "Directive: Give me the cup." \
            + "Objects: {'cup_1': {'description': 'blue cup', 'x': 0.4, 'y': 0.2, 'z': 0.1}," \
            + "'jar_1': {'description': 'jar, 'x':0.8, 'y': 0.4, 'z':0.2, 'length': 0.08, 'height': 0.12}," \
            + "'cup_2': {'description': 'red cup', 'x': 0.6, 'y': 0.1, 'z': 0.1}," \
            + "'cup_3': {'description': 'green cup', 'x': 0.5, 'y': 0.3, 'z': 0.1}," \
            + "'apple_1': {'description': 'apple', 'x': 0.5, 'y': -0.4, 'z': 0.2, 'length': 0.08, 'width': 0.08, 'height': 0.08}," \
            + "'bottle_1': {'description': 'water bottle', 'x': 0.9, 'y': 0.5, 'z': 0.2}}"
        
        example_response = "I have detected three cups: blue, red, and green. Which one would you like?\n"
        self.llm.init_history(example_query, example_response)

        self.llm_exec_pub = rospy.Publisher("/llm_commands", String, queue_size=10)
        self.llm_response_pub = rospy.Publisher("/llm_response", String, queue_size=10)
        rospy.Subscriber("/user_response", String, self.user_response_callback)


    def user_response_callback(self, msg):
        output = self.llm.query_openai("Directive: " + msg.data + 'Objects: ' + str(self.current_objects))
        commands = output.choices[0].message.content
        self.llm_response_pub.publish(commands)
        self.llm_exec_pub.publish(commands)
        
        rospy.loginfo("All commands have been published as a single message. Waiting for subscribers to process...")


    def object_sub(self, msg):

        for obj in msg.objects:
            self.current_objects[obj.id]  = {
                'description': obj.description, 
                'x': obj.x,
                'y': obj.y,
                'z': obj.z,
                'length': obj.length,
                'width':  obj.width,
                'height': obj.height,
            }


if __name__ == "__main__":
    vlm_handler = VLMHandler()
    rospy.spin()
