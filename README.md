# socialrobot_relocation

# 1. rearrange_task_planner

# 2. Package summary
: This package determines the object to be relocated in order to clear all obstacles that prevent grasping a target object in clutter.<br> 
 2.1 Maintainer status : maintained<br>
 2.2 Maintainer :<br> 
 - Sang Hun Cheong (welovehun@kist.re.kr)<br>
 - Jinhwi Lee (jinhooi@kist.re.kr)<br>
 - Changjoo Nam (cjnam@kist.re.kr)<br>
 
 2.3 Author :<br> 
 - Sang Hun Cheong (welovehun@kist.re.kr)<br>

# 3. Overview
 : This package determines the rearrange position of the object to be relocated.
 
# 4. Hardware requirements
 : The package does not require any hardware device.

# 5. Quick start 
 : Install the package through catkin build system. 

### Example Input test
```
#!/usr/bin/env python

import rospy
from rearrange_node.srv._rearrange_env_srv import *
from rearrange_node.msg._env_object_info_msg import *

def example():
    rospy.wait_for_service('rearrange_srv')
    try:
        f_check_srv = rospy.ServiceProxy('rearrange_srv', rearrange_env_srv)
        pub_msg = rearrange_env_srvRequest()

        pub_msg.target.object_name = ['target']
        pub_msg.target.object_position.x, pub_msg.target.object_position.y, pub_msg.target.object_position.z = 0.9, 0.05, 1.0 + 0.1
        pub_msg.target.object_orientation.x, pub_msg.target.object_orientation.y, pub_msg.target.object_orientation.z, pub_msg.target.object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.target.object_scale.x, pub_msg.target.object_scale.y, pub_msg.target.object_scale.z = 0.06, 0.06, 0.2

        for i in range(3):
            obs_tmp=env_object_info_msg()
            pub_msg.objects.append(obs_tmp)

        pub_msg.objects[0].object_name = ['obj1']
        pub_msg.objects[0].object_position.x, pub_msg.objects[0].object_position.y, pub_msg.objects[0].object_position.z = 0.75, 0.1, 1.0 + 0.1
        pub_msg.objects[0].object_orientation.x, pub_msg.objects[0].object_orientation.y, pub_msg.objects[0].object_orientation.z, pub_msg.objects[0].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[0].object_scale.x, pub_msg.objects[0].object_scale.y, pub_msg.objects[0].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.objects[1].object_name = ['obj2']
        pub_msg.objects[1].object_position.x, pub_msg.objects[1].object_position.y, pub_msg.objects[1].object_position.z = 0.75, 0.0, 1.0 + 0.1
        pub_msg.objects[1].object_orientation.x, pub_msg.objects[1].object_orientation.y, pub_msg.objects[1].object_orientation.z, pub_msg.objects[1].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[1].object_scale.x, pub_msg.objects[1].object_scale.y, pub_msg.objects[1].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.objects[2].object_name = ['obj3']
        pub_msg.objects[2].object_position.x, pub_msg.objects[2].object_position.y, pub_msg.objects[2].object_position.z = 0.75, -0.1, 1.0 + 0.1
        pub_msg.objects[2].object_orientation.x, pub_msg.objects[2].object_orientation.y, pub_msg.objects[2].object_orientation.z, pub_msg.objects[2].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[2].object_scale.x, pub_msg.objects[2].object_scale.y, pub_msg.objects[2].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.workspace.object_name = ['table']
        pub_msg.workspace.object_position.x, pub_msg.workspace.object_position.y, pub_msg.workspace.object_position.z = 0.7, 0.0, 0.5
        pub_msg.workspace.object_orientation.x, pub_msg.workspace.object_orientation.y, pub_msg.workspace.object_orientation.z, pub_msg.workspace.object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.workspace.object_scale.x, pub_msg.workspace.object_scale.y, pub_msg.workspace.object_scale.z = 0.5, 0.8, 1.0

        resp1 = f_check_srv(pub_msg)
        # f_ori[0]
        print resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Example"
    example()

```
### Return
```
/usr/bin/python2.7 ~/catkin_ws/src/rearrange_node/src/test_module.py
Example
object_name: [obj1]
rearrange_positions: 
  - 
    x: 0.62
    y: 0.31
    z: 1.1
  - 
    x: 0.73
    y: 0.28
    z: 1.1
  - 
    x: 0.66
    y: -0.07
    z: 1.1
  - 
    x: 0.88
    y: 0.26
    z: 1.1
  - 
    x: 0.84
    y: -0.19
    z: 1.1
  - 
    x: 0.81
    y: -0.33
    z: 1.1
  - 
    x: 0.51
    y: 0.04
    z: 1.1

Process finished with exit code 0
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
