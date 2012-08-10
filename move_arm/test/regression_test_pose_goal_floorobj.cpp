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
#include <arm_control_msgs/TrajectoryStart.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <gtest/gtest.h>

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;

void spinThread()
{
  ros::spin();
}

bool sendTuckArm()
{
  arm_control_msgs::TrajectoryStart::Request  req;
  arm_control_msgs::TrajectoryStart::Response res;

  ros::NodeHandle nh;
  ros::ServiceClient trajectory_start_client  = nh.serviceClient<arm_control_msgs::TrajectoryStart>("/r_arm_joint_waypoint_controller/TrajectoryStart");

  std::vector<std::string> names(7);
  names[0] = "r_shoulder_pan_joint";
  names[1] = "r_shoulder_lift_joint";
  names[2] = "r_upper_arm_roll_joint";
  names[3] = "r_elbow_flex_joint";
  names[4] = "r_forearm_roll_joint";
  names[5] = "r_wrist_flex_joint";
  names[6] = "r_wrist_roll_joint";

  req.trajectory.header.frame_id = "base_link";
  req.trajectory.header.stamp = ros::Time();
  req.trajectory.joint_names = names;
  req.trajectory.set_points_size(2);
  req.request_timing = 0;
  req.trajectory.points[0].set_positions_size(names.size());
  req.trajectory.points[0].positions[0] = -0.4;
  req.trajectory.points[0].positions[1] = 0.0;
  req.trajectory.points[0].positions[2] = 0.0;
  req.trajectory.points[0].positions[3] = -2.25;
  req.trajectory.points[0].positions[4] = 0;
  req.trajectory.points[0].positions[5] = 0;
  req.trajectory.points[0].positions[6] = 0;
  req.trajectory.points[1].set_positions_size(names.size());
  req.trajectory.points[1].positions[0] = -0.01;
  req.trajectory.points[1].positions[1] = 0.8;
  req.trajectory.points[1].positions[2] = -1.2;
  req.trajectory.points[1].positions[3] = -1.4;
  req.trajectory.points[1].positions[4] = 1.35;
  req.trajectory.points[1].positions[5] = -0.18;
  req.trajectory.points[1].positions[6] = 0.31;

  if (trajectory_start_client.call(req,res))
  {
    if (res.trajectory_id < 0)
    {
      ROS_ERROR("Invalid trajectory id: %d", res.trajectory_id);
      return false;
    }
    ROS_INFO("Sent trajectory %d to controller", res.trajectory_id);
    return true;
  }
  else
  {
    ROS_ERROR("Unable to start trajectory controller");
    return false;
  }
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

  goalA.group_name = "right_arm";
  goalA.num_planning_attempts = 1;
  private_handle.param<std::string>("planner_id",goalA.planner_id,std::string("chomp_planner_longrange"));
  private_handle.param<std::string>("planner_service_name",goalA.planner_service_name,std::string("/chomp_planner_longrange/plan_path"));

  goalA.allowed_planning_time = 10.0;
    
  goalA.goal_constraints.set_position_constraints_size(1);
  goalA.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
    
  goalA.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goalA.goal_constraints.position_constraints[0].position.x = 0.15;
  goalA.goal_constraints.position_constraints[0].position.y = -0.95;
  goalA.goal_constraints.position_constraints[0].position.z = 0;
    
  goalA.goal_constraints.position_constraints[0].constraint_region_shape.type = geometry_primitives::Object::BOX;
  goalA.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

  goalA.goal_constraints.position_constraints[0].constraint_region_pose.position.x = 0.15;
  goalA.goal_constraints.position_constraints[0].constraint_region_pose.position.y = -0.95;
  goalA.goal_constraints.position_constraints[0].constraint_region_pose.position.z = 0.0;

  goalA.goal_constraints.position_constraints[0].constraint_region_pose.orientation.x = 0.0;
  goalA.goal_constraints.position_constraints[0].constraint_region_pose.orientation.y = 0.0;
  goalA.goal_constraints.position_constraints[0].constraint_region_pose.orientation.z = 0.0;
  goalA.goal_constraints.position_constraints[0].constraint_region_pose.orientation.w = 1.0;

  goalA.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.goal_constraints.set_orientation_constraints_size(1);
  goalA.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";    
  goalA.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.goal_constraints.orientation_constraints[0].orientation.x = 0.0;
  goalA.goal_constraints.orientation_constraints[0].orientation.y = 0.0;
  goalA.goal_constraints.orientation_constraints[0].orientation.z = -0.7071;
  goalA.goal_constraints.orientation_constraints[0].orientation.w = 0.7071;
    
  goalA.goal_constraints.orientation_constraints[0].constraint_region_shape.type = geometry_primitives::Object::BOX;
  goalA.goal_constraints.orientation_constraints[0].constraint_region_shape.dimensions.push_back(0.2);
  goalA.goal_constraints.orientation_constraints[0].constraint_region_shape.dimensions.push_back(0.2);
  goalA.goal_constraints.orientation_constraints[0].constraint_region_shape.dimensions.push_back(0.2);


  goalA.goal_constraints.orientation_constraints[0].constraint_region_pose.position.x = 0.0;
  goalA.goal_constraints.orientation_constraints[0].constraint_region_pose.position.y = 0.0;
  goalA.goal_constraints.orientation_constraints[0].constraint_region_pose.position.z = 0.0;

  goalA.goal_constraints.orientation_constraints[0].constraint_region_pose.orientation.x = 0.0;
  goalA.goal_constraints.orientation_constraints[0].constraint_region_pose.orientation.y = 0.0;
  goalA.goal_constraints.orientation_constraints[0].constraint_region_pose.orientation.z = -0.7071;
  goalA.goal_constraints.orientation_constraints[0].constraint_region_pose.orientation.w = 0.7071;

  goalA.goal_constraints.orientation_constraints[0].weight = 1.0;
   
  std::vector<std::string> names(7);
  names[0] = "r_shoulder_pan_joint";
  names[1] = "r_shoulder_lift_joint";
  names[2] = "r_upper_arm_roll_joint";
  names[3] = "r_elbow_flex_joint";
  names[4] = "r_forearm_roll_joint";
  names[5] = "r_wrist_flex_joint";
  names[6] = "r_wrist_roll_joint";


  int num_test_attempts = 0;
  int max_attempts = 5;
  bool success = false;


  while (nh.ok())
  {
    bool finished_within_time = false;
    sleep(5);
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(100.0));
    actionlib::SimpleClientGoalState state = move_arm.getState();
    success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (!finished_within_time && num_test_attempts < max_attempts)
    {
      move_arm.cancelAllGoals();
      ROS_INFO("Timed out achieving goal A, trying again. Trying again, attempt: %d",num_test_attempts);
      num_test_attempts++;
    }
    else
    {
      if(!success)
      {
        ROS_INFO("Action unsuccessful");
        move_arm.cancelAllGoals();
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

  ros::service::waitForService("/r_arm_joint_waypoint_controller/TrajectoryStart");
  if(!sendTuckArm())
    printf("TUCKARM ain't happening! why?\n");

  return RUN_ALL_TESTS();
}

