###### Required #####
# Options: "move_to_pose", "envelop_grasp", "release", "home", "move_to_joint_position"
string type

###### Optional depending on primitive #####
# Options: "left", "right"
string arm 

geometry_msgs/PoseStamped pose
sensor_msgs/JointState joint_state
std_msgs/Float32 duration