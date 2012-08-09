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

/* \author: Ioan Sucan, Sachin Chitta */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

#include <boost/thread.hpp>
#include <gtest/gtest.h>

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;

void spinThread()
{
  ros::spin();
}

TEST(MoveArm, goToPoseGoal)
{
  ros::NodeHandle nh;
  ros::NodeHandle private_handle("~");
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(nh, "move_right_arm");
  boost::thread spin_thread(&spinThread);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  arm_navigation_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  private_handle.param<std::string>("planner_id",goalA.motion_plan_request.planner_id,std::string(""));
  private_handle.param<std::string>("planner_service_name",goalA.planner_service_name,std::string("ompl_planning/plan_kinematic_path"));

  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);


  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "r_wrist_roll_link";
  desired_pose.pose.position.x = 0.75;
  desired_pose.pose.position.y = -0.188;
  desired_pose.pose.position.z = 0;

  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
    
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  int num_test_attempts = 0;
  int max_attempts = 5;
  bool success = false;
  while (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(100.0));
    actionlib::SimpleClientGoalState state = move_arm.getState();
    success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if ((!finished_within_time || !success) && num_test_attempts < max_attempts)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A, trying again. Trying again, attempt: %d",num_test_attempts);
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
