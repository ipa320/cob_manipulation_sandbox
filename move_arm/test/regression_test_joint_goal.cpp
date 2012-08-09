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
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(nh, "move_right_arm");
  boost::thread spin_thread(&spinThread);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  
  arm_navigation_msgs::MoveArmGoal goalB;
  std::vector<std::string> names(7);
  names[0] = "r_shoulder_pan_joint";
  names[1] = "r_shoulder_lift_joint";
  names[2] = "r_upper_arm_roll_joint";
  names[3] = "r_elbow_flex_joint";
  names[4] = "r_forearm_roll_joint";
  names[5] = "r_wrist_flex_joint";
  names[6] = "r_wrist_roll_joint";

  goalB.motion_plan_request.group_name = "right_arm";
  goalB.motion_plan_request.num_planning_attempts = 1;
  goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  private_handle.param<std::string>("planner_id",goalB.motion_plan_request.planner_id,std::string("chomp_planner_longrange"));
  private_handle.param<std::string>("planner_service_name",goalB.planner_service_name,std::string("/chomp_planner_longrange/plan_path"));
    
  goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());
  for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    // goalB.motion_plan_request.goal_constraints.joint_constraints[i].header.stamp = ros::Time::now();
    // goalB.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id = "base_link";
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }
    
  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.0;
  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2;
  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.20;
   
  int num_test_attempts = 0;
  int max_attempts = 5;
  bool success = false;
  while (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalB);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    actionlib::SimpleClientGoalState state = move_arm.getState();
    success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if ((!finished_within_time || !success) && num_test_attempts < max_attempts)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
      num_test_attempts++;
    }
    else
    {
      if(!success)
      {
        ROS_INFO("Action finished: %s",state.toString().c_str());
        move_arm.cancelGoal();
      }
      ROS_INFO("Action finished: %s",state.toString().c_str());
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
