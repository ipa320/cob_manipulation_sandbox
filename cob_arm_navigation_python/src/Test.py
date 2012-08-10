#!/usr/bin/env python
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
p.z += 0.02
o = grasp_pose.pose.orientation
o.x,o.y,o.z,o.w = quaternion_from_euler(-1.581, -0.019, 2.379)

lift_pose = deepcopy(grasp_pose)
p = lift_pose.pose.position
p.x += 0.0
p.y += 0.0
p.z += 0.03

mp = MotionPlan()
#mp += MoveArm('arm', 'pregrasp')
mp += MoveComponent('tray', 'down')
mp += MoveComponent('sdh', 'cylopen')
mp += MoveArm("arm",[grasp_pose,['sdh_grasp_link']])
mp += MoveComponent('sdh', 'cylclosed')
mp += AttachObject('arm',  'milk')
mp += EnableCollision('milk','table_ikea')
mp += MoveArm("arm",[lift_pose,['sdh_grasp_link']])
mp += ResetCollisions()
mp += MoveArm('arm', 'hold')
mp += MoveComponent('tray', 'up')
mp += MoveArm('arm', [['base_link',[0.64-0.161, -0.11+0.056,0.832+0.25],[-1.441, 0.118, -0.078]],['sdh_grasp_link']])
mp += MoveComponent('sdh', 'cylopen')
mp += DetachObject('arm',  'milk')

print mp.plan(2)
for e in mp.execute():
    print e.wait()
    
get_planning_scene_interface().reset()
