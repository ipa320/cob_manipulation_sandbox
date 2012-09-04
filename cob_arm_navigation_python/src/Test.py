#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_arm_navigation_python')

import rospy
from MotionPlan import *
from MoveArm import *
from MoveHand import *
from tf.transformations import *
from copy import deepcopy
rospy.init_node("test")

from pr2_python.world_interface import WorldInterface

wi = WorldInterface()
grasp_pose = PoseStamped()
#grasp_pose.header.frame_id = "base_link"
grasp_pose.header.frame_id = "odom_combined"
grasp_pose.header.stamp = rospy.Time()
grasp_pose.pose = wi.collision_object('milk').poses[0]
p = grasp_pose.pose.position
#p.x, p.y, p.z = [-0.8,-0.2,0.9]
p.x += 0.02
p.y += 0.02
p.z += 0.02
o = grasp_pose.pose.orientation
o.x,o.y,o.z,o.w = quaternion_from_euler(-1.581, -0.019, 2.379)

lift_pose = deepcopy(grasp_pose)
p = lift_pose.pose.position
p.x += 0.0
p.y += -0.05
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

#mp += MoveArm('arm', 'home')
#mp += MoveComponent('tray', 'up')
#mp += MoveArm('arm', 'hold')
mp += MoveArm('arm', [['base_link',[0.64-0.161, -0.11+0.056,0.832+0.3],[-1.441, 0.118, -0.078]],['sdh_grasp_link']])
mp += MoveComponent('tray', 'up')
mp += MoveComponent('sdh', 'cylopen')
mp += DetachObject('arm',  'milk')
mp += EnableCollision('milk','tray_link')
#mp += MoveArm('arm', 'home')
mp += MoveArm('arm', [['base_link',[0.64-0.161, -0.11+0.056,0.832+0.22],[-1.441, 0.118, -0.078]],['sdh_grasp_link']])
mp += MoveComponent('sdh', 'cylclosed')
mp += AttachObject('arm',  'milk')
mp += EnableCollision('milk','tray_link')
mp += MoveArm('arm', [['base_link',[0.64-0.161, -0.11+0.056,0.832+0.5],[-1.441, 0.118, -0.078]],['sdh_grasp_link']])
mp += ResetCollisions()
#mp += MoveArm('arm', 'home')
mp += MoveArm("arm",[lift_pose,['sdh_grasp_link']])
mp += MoveComponent('sdh', 'cylopen')
mp += DetachObject('arm',  'milk')
mp += MoveArm('arm', 'folded')


planning_res = mp.plan(2)
print planning_res
if planning_res.success:
    print "For execution please press ENTER!"
    raw_input()
    for e in mp.execute():
        exec_res = e.wait()
        print exec_res
        if not exec_res.success:
            print "Execution failed"
            break
else:
    print "Will not execute, since planning failed"
    
get_planning_scene_interface().reset()
