import roslib#
roslib.load_manifest('cob_arm_navigation_python')

import rospy
from MotionPlan import *
from MoveArm import *
from MoveHand import *
from tf.transformations import *
from copy import deepcopy
rospy.init_node("test")

grasp_pose = PoseStamped()
grasp_pose.header.frame_id = "odom_combined"
grasp_pose.header.stamp = rospy.Time()

p = grasp_pose.pose.position
p.x, p.y, p.z = [-0.8,-0.2,0.9]
p.x += 0.02
p.y += 0.02
p.z += 0.05
#grasp_pose.pose.position.y+=0.0
#grasp_pose.pose.position.x+=0.0
o = grasp_pose.pose.orientation
o.x,o.y,o.z,o.w = quaternion_from_euler(-1.581, -0.019, 2.379)

lift_pose = deepcopy(grasp_pose)
p = lift_pose.pose.position
p.x, p.y, p.z = [-0.8,-0.2,0.9]
p.x += 0
p.y += 0
p.z += 0.10

mp = MotionPlan()
mp += MoveArm('arm', 'pregrasp')
mp += MoveHand('sdh', 'cylopen')
mp += MoveArm("arm",[grasp_pose,['sdh_grasp_link']])
mp += MoveHand('sdh', 'cylclosed')
mp += AttachObject('sdh',  'milk')
#mp += MoveArm("arm",[lift_pose,['sdh_grasp_link']])
mp += MoveArm('arm', 'pregrasp')
mp += MoveArm('arm', 'hold')

print mp.plan()
for e in mp.execute():
    print e.wait()
