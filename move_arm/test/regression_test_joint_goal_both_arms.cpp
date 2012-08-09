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

/* \author: Ioan Sucan, Sachin chitta*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

void spinThread()
{
  ros::spin();
}

TEST(MoveArm, goToJointGoal)
{
  ros::NodeHandle nh;
  ros::NodeHandle private_handle("~");
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_right_arm(nh, "move_right_arm");
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_left_arm(nh, "move_left_arm");
  boost::thread spin_thread(&spinThread);

  move_right_arm.waitForServer();
  move_left_arm.waitForServer();
  ROS_INFO("Connected to servers");
  
  arm_navigation_msgs::MoveArmGoal goalRight;
  std::vector<std::string> right_names(7);
  right_names[0] = "r_shoulder_pan_joint";
  right_names[1] = "r_shoulder_lift_joint";
  right_names[2] = "r_upper_arm_roll_joint";
  right_names[3] = "r_elbow_flex_joint";
  right_names[4] = "r_forearm_roll_joint";
  right_names[5] = "r_wrist_flex_joint";
  right_names[6] = "r_wrist_roll_joint";

  arm_navigation_msgs::MoveArmGoal goalLeft;
  std::vector<std::string> left_names(7);
  left_names[0] = "l_shoulder_pan_joint";
  left_names[1] = "l_shoulder_lift_joint";
  left_names[2] = "l_upper_arm_roll_joint";
  left_names[3] = "l_elbow_flex_joint";
  left_names[4] = "l_forearm_roll_joint";
  left_names[5] = "l_wrist_flex_joint";
  left_names[6] = "l_wrist_roll_joint";

  goalRight.motion_plan_request.group_name = "right_arm";
  goalRight.motion_plan_request.num_planning_attempts = 1;
  goalRight.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  private_handle.param<std::string>("planner_id",goalRight.motion_plan_request.planner_id,std::string("chomp_planner_longrange"));
  private_handle.param<std::string>("planner_service_name",goalRight.planner_service_name,std::string("/chomp_planner_longrange/plan_path"));
    
  goalRight.motion_plan_request.goal_constraints.joint_constraints.resize(right_names.size());
  for (unsigned int i = 0 ; i < goalRight.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    // goalRight.motion_plan_request.goal_constraints.joint_constraints[i].header.stamp = ros::Time::now();
    // goalRight.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id = "base_link";
    goalRight.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = right_names[i];
    goalRight.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalRight.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalRight.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }
    
  goalRight.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.0;
  goalRight.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2;
  goalRight.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.20;

  
   
  goalLeft.motion_plan_request.group_name = "left_arm";
  goalLeft.motion_plan_request.num_planning_attempts = 1;
  goalLeft.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  private_handle.param<std::string>("planner_id",goalLeft.motion_plan_request.planner_id,std::string("chomp_planner_longrange"));
  private_handle.param<std::string>("planner_service_name",goalLeft.planner_service_name,std::string("/chomp_planner_longrange/plan_path"));
    
  goalLeft.motion_plan_request.goal_constraints.joint_constraints.resize(left_names.size());
  for (unsigned int i = 0 ; i < goalLeft.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    // goalLeft.motion_plan_request.goal_constraints.joint_constraints[i].header.stamp = ros::Time::now();
    // goalLeft.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id = "base_link";
    goalLeft.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = left_names[i];
    goalLeft.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalLeft.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalLeft.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }
    
  goalLeft.motion_plan_request.goal_constraints.joint_constraints[0].position = 2.0;
  goalLeft.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2;
  goalLeft.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.20;

  int num_test_attempts = 0;
  int max_attempts = 5;
  bool success = false;
  while (nh.ok())
  {
    bool right_finished_within_time = false;
    bool left_finished_within_time = false;
    move_right_arm.sendGoal(goalRight);
    move_left_arm.sendGoal(goalLeft);
   
    right_finished_within_time = move_right_arm.waitForResult(ros::Duration(200.0));
    left_finished_within_time = move_left_arm.waitForResult(ros::Duration(10.0));
    
    actionlib::SimpleClientGoalState right_state = move_right_arm.getState();
    actionlib::SimpleClientGoalState left_state = move_left_arm.getState();
    success = (right_state == actionlib::SimpleClientGoalState::SUCCEEDED && left_state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if(!right_finished_within_time || right_state != actionlib::SimpleClientGoalState::SUCCEEDED) {
      move_right_arm.cancelGoal();
      ROS_INFO("Right arm timed out achieving goal");
    }
    if(!left_finished_within_time || left_state != actionlib::SimpleClientGoalState::SUCCEEDED) {
      move_left_arm.cancelGoal();
      ROS_INFO("Left arm timed out achieving goal");
    }
    if(!success) {
      num_test_attempts++;
      if(num_test_attempts > max_attempts) {
        break;
      }
    } else {
      break;
    }
  }
  EXPECT_TRUE(success);
  ros::shutdown();
  spin_thread.join();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "move_arm_regression_test");
  return RUN_ALL_TESTS();
}
