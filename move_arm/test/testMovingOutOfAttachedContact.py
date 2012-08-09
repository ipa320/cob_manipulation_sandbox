#! /usr/bin/env python

PKG = 'move_arm'

import roslib; roslib.load_manifest(PKG)
import rospy
import arm_navigation_msgs.srv
import sys
import unittest
import actionlib
import actionlib_msgs
import math

import sensor_msgs.msg
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject, AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose, PointStamped
from move_arm_msgs.msg import MoveArmGoal, MoveArmAction
from tf import TransformListener
from arm_navigation_msgs.msg import JointConstraint

class TestMotionExecutionBuffer(unittest.TestCase):

    def setUp(self):

        self.tf = TransformListener()
        
        self.move_arm_action_client = actionlib.SimpleActionClient("move_right_arm", MoveArmAction)



        att_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)
        obj_pub = rospy.Publisher('collision_object',CollisionObject)
        
        rospy.init_node('test_motion_execution_buffer')
        
        #let everything settle down
        rospy.sleep(5.)
        
        obj1 = CollisionObject()
    
        obj1.header.stamp = rospy.Time.now()-rospy.Duration(.1)
        obj1.header.frame_id = "base_footprint"
        obj1.id = "obj2";
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj1.shapes = [Shape() for _ in range(3)]
        obj1.poses = [Pose() for _ in range(3)]

        obj1.shapes[0].type = Shape.BOX
        obj1.shapes[0].dimensions = [float() for _ in range(3)]
        obj1.shapes[0].dimensions[0] = .5
        obj1.shapes[0].dimensions[1] = 1.0
        obj1.shapes[0].dimensions[2] = .2
        obj1.poses[0].position.x = .95
        obj1.poses[0].position.y = -.25
        obj1.poses[0].position.z = .62
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1

        obj1.shapes[2].type = Shape.BOX
        obj1.shapes[2].dimensions = [float() for _ in range(3)]
        obj1.shapes[2].dimensions[0] = .5
        obj1.shapes[2].dimensions[1] = .1
        obj1.shapes[2].dimensions[2] = 1.0
        obj1.poses[2].position.x = .95
        obj1.poses[2].position.y = -.14
        obj1.poses[2].position.z = 1.2
        obj1.poses[2].orientation.x = 0
        obj1.poses[2].orientation.y = 0
        obj1.poses[2].orientation.z = 0
        obj1.poses[2].orientation.w = 1

        obj1.shapes[1].type = Shape.BOX
        obj1.shapes[1].dimensions = [float() for _ in range(3)]
        obj1.shapes[1].dimensions[0] = .5
        obj1.shapes[1].dimensions[1] = .1
        obj1.shapes[1].dimensions[2] = 1.0
        obj1.poses[1].position.x = .95
        obj1.poses[1].position.y = .12
        obj1.poses[1].position.z = 1.2
        obj1.poses[1].orientation.x = 0
        obj1.poses[1].orientation.y = 0
        obj1.poses[1].orientation.z = 0
        obj1.poses[1].orientation.w = 1
        
        obj_pub.publish(obj1)

        att_object = AttachedCollisionObject()
        att_object.object.header.stamp = rospy.Time.now()
        att_object.object.header.frame_id = "r_gripper_r_finger_tip_link"
        att_object.link_name = "r_gripper_r_finger_tip_link"
        att_object.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        att_object.object = CollisionObject();

        att_object.object.header.stamp = rospy.Time.now()
        att_object.object.header.frame_id = "r_gripper_r_finger_tip_link"
        att_object.object.id = "pole"
        att_object.object.shapes = [Shape() for _ in range(1)]
        att_object.object.shapes[0].type = Shape.CYLINDER
        att_object.object.shapes[0].dimensions = [float() for _ in range(2)]
        att_object.object.shapes[0].dimensions[0] = .02
        att_object.object.shapes[0].dimensions[1] = .1
        att_object.object.poses = [Pose() for _ in range(1)]
        att_object.object.poses[0].position.x = -.02
        att_object.object.poses[0].position.y = .04
        att_object.object.poses[0].position.z = 0
        att_object.object.poses[0].orientation.x = 0
        att_object.object.poses[0].orientation.y = 0
        att_object.object.poses[0].orientation.z = 0
        att_object.object.poses[0].orientation.w = 1

        att_pub.publish(att_object)

        rospy.sleep(5.0)        

    def testMotionExecutionBuffer(self):
        
        global padd_name
        global extra_buffer
        
        #too much trouble to read for now
        allow_padd = .05#rospy.get_param(padd_name)
        

        joint_names = ['%s_%s' % ('r', j) for j in ['shoulder_pan_joint', 'shoulder_lift_joint', 'upper_arm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']]
        goal = MoveArmGoal()

        goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]

        goal.motion_plan_request.group_name = "right_arm"
        goal.motion_plan_request.num_planning_attempts = 1
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.)
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "chomp_planner_longrange/plan_path"
        goal.disable_collision_monitoring = True;

        r = rospy.Rate(10)

        goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]
        for z in range(6):
            for i in range(len(joint_names)):
                goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names[i]
                goal.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0
                goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.08
                goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.08

            if(z%2==0):
                goal.motion_plan_request.goal_constraints.joint_constraints[0].position = -1.0
                goal.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2
                goal.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.2
            else:
                goal.motion_plan_request.goal_constraints.joint_constraints[0].position = .22
                goal.motion_plan_request.goal_constraints.joint_constraints[1].position = .51
                goal.motion_plan_request.goal_constraints.joint_constraints[2].position = .05
                goal.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.41
                goal.motion_plan_request.goal_constraints.joint_constraints[4].position = -.06
                goal.motion_plan_request.goal_constraints.joint_constraints[5].position = -.1
                goal.motion_plan_request.goal_constraints.joint_constraints[6].position = 0.0

            for x in range(3):

                self.move_arm_action_client.send_goal(goal)
            
                while True:
                    cur_state = self.move_arm_action_client.get_state()
                    if(cur_state != actionlib_msgs.msg.GoalStatus.ACTIVE and
                       cur_state != actionlib_msgs.msg.GoalStatus.PENDING):
                        break
                    r.sleep()
                
                end_state = self.move_arm_action_client.get_state()
             
                if(end_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED): break
                
            final_state = self.move_arm_action_client.get_state()

            self.assertEqual(final_state,  actionlib_msgs.msg.GoalStatus.SUCCEEDED)

if __name__ == '__main__':

    import rostest
    rostest.unitrun('test_motion_execution_buffer', 'test_motion_execution_buffer', TestMotionExecutionBuffer)


    
