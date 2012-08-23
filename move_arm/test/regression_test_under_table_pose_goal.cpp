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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>
#include <arm_navigation_msgs/MakeStaticCollisionMapAction.h>

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;

unsigned int REPS_TO_TRY = 10;


void spinThread()
{
  ros::spin();
}

TEST(MoveArm, goToPoseGoal)
{

  ros::NodeHandle nh;
  ros::NodeHandle private_handle("~");

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);

  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(nh, "move_right_arm");
  actionlib::SimpleActionClient<arm_navigation_msgs::MakeStaticCollisionMapAction> make_static_map(nh, "make_static_collision_map");

  boost::thread spin_thread(&spinThread);
  
  move_arm.waitForServer();
  make_static_map.waitForServer();
  ROS_INFO("Connected to servers");

  //push the table and legs into the collision space
  arm_navigation_msgs::CollisionObject table_object;
  table_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  table_object.header.frame_id = "base_link";
  table_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = 1.0;
  object.dimensions[1] = 1.0;
  object.dimensions[2] = 0.05;
  geometry_msgs::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0;
  pose.position.z = .5;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  table_object.shapes.push_back(object);
  table_object.poses.push_back(pose);

  table_object.id = "table";
  object_in_map_pub_.publish(table_object);

  arm_navigation_msgs::CollisionObject leg_object;
  leg_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  leg_object.header.frame_id = "base_link";
  leg_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape l_object;
  l_object.type = arm_navigation_msgs::Shape::BOX;
  l_object.dimensions.resize(3);
  l_object.dimensions[0] = 0.05;
  l_object.dimensions[1] = 0.05;
  l_object.dimensions[2] = 0.5;
  geometry_msgs::Pose l_pose;
  l_pose.position.x = .5;
  l_pose.position.y = .5;
  l_pose.position.z = .25;
  l_pose.orientation.x = 0;
  l_pose.orientation.y = 0;
  l_pose.orientation.z = 0;
  l_pose.orientation.w = 1;
  leg_object.shapes.push_back(l_object);
  leg_object.poses.push_back(l_pose);
  l_pose.position.x = .5;
  l_pose.position.y = -.5;
  l_pose.position.z = .25;
  leg_object.shapes.push_back(l_object);
  leg_object.poses.push_back(l_pose);
  leg_object.id = "legs";
  object_in_map_pub_.publish(leg_object);

  ros::Duration(10.0).sleep();

  std::vector<std::string> names(7);
  names[0] = "r_shoulder_pan_joint";
  names[1] = "r_shoulder_lift_joint";
  names[2] = "r_upper_arm_roll_joint";
  names[3] = "r_elbow_flex_joint";
  names[4] = "r_forearm_roll_joint";
  names[5] = "r_wrist_flex_joint";
  names[6] = "r_wrist_roll_joint";

  arm_navigation_msgs::MoveArmGoal goalA, goalB;
  goalB.motion_plan_request.group_name = "right_arm";
  goalB.motion_plan_request.num_planning_attempts = 1;
  goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  private_handle.param<std::string>("planner_id",goalB.motion_plan_request.planner_id,std::string(""));
  private_handle.param<std::string>("planner_service_name",goalB.planner_service_name,std::string("/ompl_planning/plan_kinematic_path"));
    
  goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());
  for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    //    goalB.motion_plan_request.goal_constraints.joint_constraints[i].header.stamp = ros::Time::now();
    //    goalB.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id = "base_link";
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }
    
// Joint r_shoulder_pan_joint val -0.453899
// [ INFO] [61.024000000]: Joint r_shoulder_lift_joint val 0.31966
// [ INFO] [61.024000000]: Joint r_upper_arm_roll_joint val -0.142258
// [ INFO] [61.024000000]: Joint r_elbow_flex_joint val -0.563305
// [ INFO] [61.024000000]: Joint r_forearm_roll_joint val -0.00941518
// [ INFO] [61.024000000]: Joint r_wrist_flex_joint val -0.744197
// [ INFO] [61.024000000]: Joint r_wrist_roll_joint val 0.410848


  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -1.5;
  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2;
  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.20;

//   goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -.453899;
//   goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = .31966;
//   goalB.motion_plan_request.goal_constraints.joint_constraints[2].position = -0.142258;
//   goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.563305;
//   goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = -0.00941518;
//   goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.744197;
//   goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = .410848;

  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -1.05023;
  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = -0.0454;
  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position = -2.81824;
  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -1.50497;
  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = 1.40266;
  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.476981;
  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = .477598;


  goalA.motion_plan_request.planner_id = "";
  goalA.planner_service_name ="ompl_planning/plan_kinematic_path"; 

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  goalA.motion_plan_request.goal_constraints.position_constraints.resize(1);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "base_link";
    
  goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = 0.6;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = 0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = .35;
    
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;

  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "base_link";    
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = 1.0;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 3.14;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 3.14;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 3.14;
    
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

  for(unsigned int i = 0; i < REPS_TO_TRY; i++) {
    
    if (nh.ok())
    {
      bool finished_within_time = false;
      move_arm.sendGoal(goalA);
      finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
      EXPECT_TRUE(finished_within_time);
      if (!finished_within_time)
      {
        move_arm.cancelGoal();
        ROS_INFO("Timed out achieving goal A");
      }
      else
      {
        actionlib::SimpleClientGoalState state = move_arm.getState();
        bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        ASSERT_TRUE(success);
        ROS_INFO("Action finished: %s",state.toString().c_str());
        ASSERT_TRUE(state == actionlib::SimpleClientGoalState::SUCCEEDED);
      }
    }

  //   arm_navigation_msgs::MakeStaticCollisionMapGoal static_map_goal;
    
//     static_map_goal.cloud_source = "full_cloud_filtered";
//     static_map_goal.number_of_clouds = 2;

//     make_static_map.sendGoal(static_map_goal);

//     make_static_map.waitForResult(ros::Duration(10.0));

//     ROS_INFO("Should have static map");

    if (nh.ok())
    {
      bool finished_within_time = false;
      move_arm.sendGoal(goalB);
      finished_within_time = move_arm.waitForResult(ros::Duration(100.0));
      EXPECT_TRUE(finished_within_time);
      if (!finished_within_time)
      {
        move_arm.cancelAllGoals();
        ROS_INFO("Timed out achieving goal B");
      }
      else
      {
        actionlib::SimpleClientGoalState state = move_arm.getState();
        bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        ASSERT_TRUE(success);
        ROS_INFO("Action finished: %s",state.toString().c_str());
        ASSERT_TRUE(state == actionlib::SimpleClientGoalState::SUCCEEDED);
      }
    }
  }
  ros::shutdown();
  spin_thread.join();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "move_arm_regression_test");
  return RUN_ALL_TESTS();
}
