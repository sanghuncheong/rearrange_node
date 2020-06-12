# socialrobot_relocation

# 1. rearrange_task_planner

# 2. Package summary
: This package determines the object to be relocated in order to clear all obstacles that prevent grasping a target object in clutter. It can be used for three cases varying depending on the degree of uncertainty in object recognition: 1) all objects are detected, 2) the target object is detected but some other objects are not, and 3) some objects including the target object are undetected.
 2.1 Maintainer status : maintained
 2.2 Maintainer : Sang Hun Cheong (welovehun@kist.re.kr), Jinhwi Lee (jinhooi@kist.re.kr), Changjoo Nam (cjnam@kist.re.kr)
 2.3 Author : Sang Hun Cheong (welovehun@kist.re.kr)

# 3. Overview
 : This package determines the object to be relocated in order to clear all obstacles that prevent grasping a target object in clutter. It works for the cases where i) all objects are known, ii) some objects including the target are unknown, and iii) some objects including the target are unknown. The input is the 2D coordinates of all known movable objects and immovable objects (e.g., walls). Before every relocation action of the robot, this information is obtained from the service node system_manager. The package finds the object to be relocated. The information of the object (ID and 2D coordinates) is sent to the client node system_manager. 

# 4. Hardware requirements
 : The package does not require any hardware device.

# 5. Quick start 
 : Install the package through catkin build system. 

===============================================================
```
Example input (10 objects, target ID = 8)
robot_height = 0.075
robot_pose = [0.32299999999999995, -0.06575]
target_id = 8
N = 10
R = [0.025, 0.026, 0.03, 0.028, 0.026, 0.025, 0.03, 0.03, 0.029, 0.025, 0.03]
H = [0.073, 0.071, 0.068, 0.07, 0.071, 0.069, 0.07, 0.073, 0.066, 0.066, 0.075]
X = [0.475, 0.306, 0.475, 0.36405000000000004, 0.306, 0.17099999999999999, 0.43005000000000004, 0.20405, 0.20900000000000002, 0.17099999999999999]
Y = [0.19475, 0.14075000000000001, 0.36975, 0.26625, 0.40975000000000006, 0.16075, 0.27425, 0.28725, 0.04225, 0.38375000000000004]
x_min =0.06299999999999999
x_max =0.583
y_min =-0.06575
y_max =0.51775
# Example run
```
```
import rospy
from relocate_planner.srv._relocate_env_srv import *
rospy.wait_for_service('relocate_srv')
reloc_srv = rospy.ServiceProxy('relocate_srv', relocate_env_srv)
pub_msg = relocate_env_srvRequest()
pub_msg.robot_height = robot_height
pub_msg.robot_pose = robot_pose
pub_msg.target_id = target_id
pub_msg.N = N
pub_msg.R = R
pub_msg.H = H
pub_msg.X = X
pub_msg.Y = Y
pub_msg.x_min = x_min
pub_msg.x_max = x_max
pub_msg.y_min = y_min
pub_msg.y_max = y_max
ret = reloc_srv(pub_msg)
accessibility = ret.accessibility
relocate_id= ret.relocate_id
relocate_coordinates = ret.relocate_coordinates
print('Target accessibility (-1=unaccessible, 0=undetected, 1=accessible): %d' % accessibility)print('Relocate Object %d at (%f, %f)' % (relocate_id, relocate_coordinates[0], relocate_coordinates[1]))
```

# 6. Input/Service Request
○ env_object_info_msg.msg<br>
> Header header : Standard metadata for higher-level stamped data types.<br>
> string[] object_name : the name of the object.<br>
> geometry_msgs/Point object_position : the position of the object.<br>
> geometry_msgs/Quaternion object_orientation : the orientation of the object in quaternions(x, y, z, w).<br>
> geometry_msgs/Vector3 object_scale : the scale of the object in each x, y, z-axis.<br>

○ rearrange_env_srv.srv<br>
> Service requests<br>
>> env_object_info_msg workspace : the workspace object (ex.table).<br>
>> env_object_info_msg target : the target object on the workspace object.<br>
>> env_object_info_msg[] objects : the obstacles on the workspace object.<br>

# 7. Output/Service Response
○ rearrange_env_srv.srv<br>
> Service response<br>
>> string[] object_name : the object to be relocated.<br>
>> geometry_msgs/Point[] rearrange_positions : the list of positions that the object to be rearranged.<br>

# 8. Parameters
N/A
