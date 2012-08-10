/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *********************************************************************/

/* \author: Ioan Sucan */

#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm/MoveArmAction.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <planning_models/kinematic_state.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <gtest/gtest.h>

void setupGoal(const std::vector<std::string> &names, move_arm::MoveArmGoal &goal)
{
    goal.goal_constraints.joint_constraint.resize(names.size());
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size(); ++i)
    {
	goal.goal_constraints.joint_constraint[i].header.stamp = ros::Time::now();
	goal.goal_constraints.joint_constraint[i].header.frame_id = "/base_link";
	goal.goal_constraints.joint_constraint[i].joint_name = names[i];
	goal.goal_constraints.joint_constraint[i].value.resize(1);
	goal.goal_constraints.joint_constraint[i].tolerance_above.resize(1);
	goal.goal_constraints.joint_constraint[i].tolerance_below.resize(1);
	goal.goal_constraints.joint_constraint[i].value[0] = 0.0;
	goal.goal_constraints.joint_constraint[i].tolerance_below[0] = 0.0;
	goal.goal_constraints.joint_constraint[i].tolerance_above[0] = 0.0;
    }

    goal.contacts.resize(1);
    goal.contacts[0].links.push_back("r_gripper_l_finger_link");
    goal.contacts[0].links.push_back("r_gripper_r_finger_link");
    goal.contacts[0].links.push_back("r_gripper_l_finger_tip_link");
    goal.contacts[0].links.push_back("r_gripper_r_finger_tip_link");
    goal.contacts[0].links.push_back("r_gripper_palm_link");
    goal.contacts[0].links.push_back("r_wrist_roll_link");
    goal.contacts[0].depth = 0.04;
    goal.contacts[0].bound.type = arm_navigation_msgs::Object::SPHERE;
    goal.contacts[0].bound.dimensions.push_back(0.5);

    goal.contacts[0].pose.header.stamp = ros::Time::now();
    goal.contacts[0].pose.header.frame_id = "/base_link";
    goal.contacts[0].pose.pose.position.x = 1;
    goal.contacts[0].pose.pose.position.y = 0;
    goal.contacts[0].pose.pose.position.z = 0.5;

    goal.contacts[0].pose.pose.orientation.x = 0;
    goal.contacts[0].pose.pose.orientation.y = 0;
    goal.contacts[0].pose.pose.orientation.z = 0;
    goal.contacts[0].pose.pose.orientation.w = 1;
}

void goalToState(const move_arm::MoveArmGoal &goal, planning_models::KinematicState &sp)
{
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size() ; ++i)
    {
	sp.setParamsJoint(&goal.goal_constraints.joint_constraint[i].value[0],
			  goal.goal_constraints.joint_constraint[i].joint_name);
    }
}

btTransform effPosition(const planning_environment::KinematicModelStateMonitor &km, const move_arm::MoveArmGoal &goal)
{
    planning_models::KinematicState sp(*km.getRobotState());
    goalToState(goal, sp);
    km.getKinematicModel()->computeTransforms(sp.getParams());
    return km.getKinematicModel()->getJoint("r_wrist_roll_joint")->after->globalTrans;
}

void diffConfig(const planning_environment::KinematicModelStateMonitor &km, move_arm::MoveArmGoal &goal)
{
  std::vector<std::string> names(7);
    names[0] = "r_shoulder_pan_joint";
    names[1] = "r_shoulder_lift_joint";
    names[2] = "r_upper_arm_roll_joint";
    names[3] = "r_elbow_flex_joint";
    names[4] = "r_forearm_roll_joint";
    names[5] = "r_wrist_flex_joint";
    names[6] = "r_wrist_roll_joint";
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size(); ++i)
    {
	std::cout << "  " << goal.goal_constraints.joint_constraint[i].joint_name << " = "
		  << goal.goal_constraints.joint_constraint[i].value[0] - km.getRobotState()->getParamsJoint(goal.goal_constraints.joint_constraint[i].joint_name)[0]
		  << std::endl;
    }

    btTransform pose1 = effPosition(km, goal);
    move_arm::MoveArmGoal temp;

    planning_models::KinematicState sp(*(km.getRobotState()));
    sp.enforceBounds();
    for (unsigned int i = 0 ; i < names.size() ; ++i)
    {
      temp.goal_constraints.joint_constraint[i].value[0] =
        sp.getParamsJoint(temp.goal_constraints.joint_constraint[i].joint_name)[0];
    }
    btTransform pose2 = effPosition(km, temp);
    std::cout << std::endl;
    double dist = pose1.getOrigin().distance(pose2.getOrigin());
    std::cout << "  -position distance: " << dist << std::endl;
    double angle = pose1.getRotation().angle(pose2.getRotation());
    std::cout << "  -rotation distance: " << angle << std::endl;
}

void spinThread()
{
  ros::spin();
}

TEST(MoveArm, goToPoseGoal)
{
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm::MoveArmAction> move_arm(nh, "move_right_arm");
  boost::thread spin_thread(&spinThread);

  nh.setParam( "/move_right_arm/long_range_only", true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  move_arm::MoveArmGoal goalA;

  //getting a monitor so that we can track the configuration of the arm
  planning_environment::RobotModels rm("robot_description");
  EXPECT_TRUE(rm.loadedModels());
    
  tf::TransformListener tf;
  planning_environment::KinematicModelStateMonitor km(&rm, &tf);
  km.waitForState();
  //should have the state at this point
  
  goalA.goal_constraints.set_pose_constraint_size(1);
  goalA.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
  goalA.goal_constraints.pose_constraint[0].pose.header.frame_id = "base_link";
  
  //-position [x, y, z]    = [0.413446, 0.0124666, 0.206998]
  //-rotation [x, y, z, w] = [0.132744, 0.888279, 0.288412, 0.331901]

  goalA.goal_constraints.pose_constraint[0].type = arm_navigation_msgs::PoseConstraint::POSITION_X 
    + arm_navigation_msgs::PoseConstraint::POSITION_Y 
    + arm_navigation_msgs::PoseConstraint::POSITION_Z
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_R 
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_P 
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_Y;
  
  goalA.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
  goalA.goal_constraints.pose_constraint[0].pose.pose.position.x = 0.413446;
  goalA.goal_constraints.pose_constraint[0].pose.pose.position.y = 0.0124666;
  goalA.goal_constraints.pose_constraint[0].pose.pose.position.z = 0.206998;
    
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.x = .132744;
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.y = .888279;
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.z = .288412;
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.w = .331901;

  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.003;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.003;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.003;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.003;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.003;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.003;

  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.005;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.005;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.01;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.005;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.005;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.005;
  
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.005;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.005;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.005;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.005;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.005;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.005;  

  goalA.goal_constraints.pose_constraint[0].orientation_importance = 0.2;

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(10.0));
    
    diffConfig(km,goalA);

    EXPECT_TRUE(finished_within_time);
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
  }

  //now we check that we actually achieved the configuration within the tolerances
  diffConfig(km,goalA);

  ros::shutdown();
  spin_thread.join();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "move_arm_regression_test");
  return RUN_ALL_TESTS();
}
