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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>

#include "regression_test_arm_helpers.h"

void spinThread()
{
  ros::spin();
}

TEST(MoveArm, goToPoseGoal)
{
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm::MoveArmAction> move_arm(nh, "move_right_arm");
  boost::thread spin_thread(&spinThread);

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
  
  //starting configuration
  //-position [x, y, z]    = [0.77025, -.18, 0.73]
  //-rotation [x, y, z, w] = [0, -0.05, 0, 0]

  goalA.goal_constraints.pose_constraint[0].type = arm_navigation_msgs::PoseConstraint::POSITION_X 
    + arm_navigation_msgs::PoseConstraint::POSITION_Y 
    + arm_navigation_msgs::PoseConstraint::POSITION_Z
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_R 
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_P 
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_Y;
  
  goalA.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
  goalA.goal_constraints.pose_constraint[0].pose.pose.position.x = 0.60;
  goalA.goal_constraints.pose_constraint[0].pose.pose.position.y = -0.25;
  goalA.goal_constraints.pose_constraint[0].pose.pose.position.z = 0.84;
    
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.x = 0;
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.y = 0;
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.z = 0;
  goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.w = 1.0;

  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.01;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.01;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.01;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.01;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.01;
  goalA.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.01;

  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.01;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.01;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.01;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.01;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.01;
  goalA.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.01;

  goalA.goal_constraints.pose_constraint[0].orientation_importance = 0.2;

  goalA.contacts.resize(2);
  goalA.contacts[0].links.push_back("r_gripper_l_finger_link");
  goalA.contacts[0].links.push_back("r_gripper_r_finger_link");
  goalA.contacts[0].links.push_back("r_gripper_l_finger_tip_link");
  goalA.contacts[0].links.push_back("r_gripper_r_finger_tip_link");
  
  goalA.contacts[0].depth = 0.04;
  goalA.contacts[0].bound.type = arm_navigation_msgs::Object::SPHERE;
  goalA.contacts[0].bound.dimensions.push_back(0.3);
  goalA.contacts[0].pose = goalA.goal_constraints.pose_constraint[0].pose;
  
  goalA.contacts[1].links.push_back("r_gripper_palm_link");
  goalA.contacts[1].links.push_back("r_wrist_roll_link");
  goalA.contacts[1].depth = 0.02;
  goalA.contacts[1].bound.type = arm_navigation_msgs::Object::SPHERE;
  goalA.contacts[1].bound.dimensions.push_back(0.2);
  goalA.contacts[1].pose = goalA.goal_constraints.pose_constraint[0].pose;
  
  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(10.0));
    
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
      EXPECT_TRUE(state == actionlib::SimpleClientGoalState::SUCCEEDED);

      double dist_pose;
      double dist_angle;
      
      diffConfig(km,goalA,dist_pose,dist_angle);

      //close enough - summed tolerances
      EXPECT_TRUE(dist_pose < .005+.005+.01);
      EXPECT_TRUE(dist_angle < .005*3);

      EXPECT_TRUE(finalStateMatchesGoal(km,goalA));
    
    }
  }

  nh.setParam( "/move_right_arm/long_range_only", true);

  move_arm::MoveArmGoal goalB;
  
  //starting configuration
  //-position [x, y, z]    = [0.77025, -.18, 0.73]
  //-rotation [x, y, z, w] = [0, -0.05, 0, 0]

  goalB.goal_constraints.set_pose_constraint_size(1);
  goalB.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
  goalB.goal_constraints.pose_constraint[0].pose.header.frame_id = "base_link";
  goalB.goal_constraints.pose_constraint[0].type = arm_navigation_msgs::PoseConstraint::POSITION_X 
    + arm_navigation_msgs::PoseConstraint::POSITION_Y 
    + arm_navigation_msgs::PoseConstraint::POSITION_Z
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_R 
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_P 
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_Y;
  
  goalB.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
  goalB.goal_constraints.pose_constraint[0].pose.pose.position.x = 0.60;
  goalB.goal_constraints.pose_constraint[0].pose.pose.position.y = -0.25;
  goalB.goal_constraints.pose_constraint[0].pose.pose.position.z = 0.5;
  
  goalB.goal_constraints.pose_constraint[0].pose.pose.orientation.x = 0;
  goalB.goal_constraints.pose_constraint[0].pose.pose.orientation.y = 0;
  goalB.goal_constraints.pose_constraint[0].pose.pose.orientation.z = 0;
  goalB.goal_constraints.pose_constraint[0].pose.pose.orientation.w = 1.0;
  
  goalB.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.05;
  goalB.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.05;
  goalB.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.05;
  goalB.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.05;
  goalB.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.05;
  goalB.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.05;

  goalB.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.05;
  goalB.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.05;
  goalB.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.05;
  goalB.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.05;
  goalB.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.05;
  goalB.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.05;

  goalB.goal_constraints.pose_constraint[0].orientation_importance = 0.2;

  goalB.path_constraints.set_pose_constraint_size(1);
  goalB.path_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
  goalB.path_constraints.pose_constraint[0].pose.header.frame_id = "base_link";
  goalB.path_constraints.pose_constraint[0].type = arm_navigation_msgs::PoseConstraint::POSITION_X 
    + arm_navigation_msgs::PoseConstraint::POSITION_Y
    + arm_navigation_msgs::PoseConstraint::POSITION_Z
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_R
    + arm_navigation_msgs::PoseConstraint::ORIENTATION_P; 
//     + arm_navigation_msgs::PoseConstraint::ORIENTATION_Y;
  goalB.path_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
  goalB.path_constraints.pose_constraint[0].pose.pose.position.x = 0.60;
  goalB.path_constraints.pose_constraint[0].pose.pose.position.y = -0.25;
  goalB.path_constraints.pose_constraint[0].pose.pose.position.z = .45;
    
  goalB.path_constraints.pose_constraint[0].pose.pose.orientation.x = 0;
  goalB.path_constraints.pose_constraint[0].pose.pose.orientation.y = 0;
  goalB.path_constraints.pose_constraint[0].pose.pose.orientation.z = 0;
  goalB.path_constraints.pose_constraint[0].pose.pose.orientation.w = 1.0;

  goalB.path_constraints.pose_constraint[0].position_tolerance_above.x = 0.1;
  goalB.path_constraints.pose_constraint[0].position_tolerance_above.y = 0.1;
  goalB.path_constraints.pose_constraint[0].position_tolerance_below.x = 0.1;
  goalB.path_constraints.pose_constraint[0].position_tolerance_below.y = 0.1;
  goalB.path_constraints.pose_constraint[0].position_tolerance_above.z = 10.0;
  goalB.path_constraints.pose_constraint[0].position_tolerance_below.z = 0.1;

  goalB.path_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.1;
  goalB.path_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.1;
//   goalB.path_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.05;
  goalB.path_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.1;
  goalB.path_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.1;
//   goalB.path_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.05;

  goalB.path_constraints.pose_constraint[0].orientation_importance = 0.4;

  goalB.contacts.resize(2);
  goalB.contacts[0].links.push_back("r_gripper_l_finger_link");
  goalB.contacts[0].links.push_back("r_gripper_r_finger_link");
  goalB.contacts[0].links.push_back("r_gripper_l_finger_tip_link");
  goalB.contacts[0].links.push_back("r_gripper_r_finger_tip_link");
  
  goalB.contacts[0].depth = 0.04;
  goalB.contacts[0].bound.type = arm_navigation_msgs::Object::SPHERE;
  goalB.contacts[0].bound.dimensions.push_back(0.3);
  goalB.contacts[0].pose = goalB.goal_constraints.pose_constraint[0].pose;
  
  goalB.contacts[1].links.push_back("r_gripper_palm_link");
  goalB.contacts[1].links.push_back("r_wrist_roll_link");
  goalB.contacts[1].depth = 0.02;
  goalB.contacts[1].bound.type = arm_navigation_msgs::Object::SPHERE;
  goalB.contacts[1].bound.dimensions.push_back(0.2);
  goalB.contacts[1].pose = goalB.goal_constraints.pose_constraint[0].pose;
  
  if (nh.ok())
  {
    move_arm.sendGoal(goalB);

    ros::Time start_time = ros::Time::now();
    ros::Duration elapsed(0.0);

    //trying ticks in case time gets wonky
    unsigned int ticks=0;

    while(1) {
      
      bool result_during_cycle = move_arm.waitForResult(ros::Duration(.2));
      
      //got some result before time out
      if(result_during_cycle) {
        break;
      }
      
      elapsed = ros::Time::now()-start_time;

      std::cout << "Time " << elapsed.toSec() << std::endl;

      //checking if we've gone over max time - if so bail
      if(elapsed.toSec() > 10.0 || ticks++ > 50) break;

      goalB.path_constraints.pose_constraint[0].pose.pose.orientation.x = 0;
      goalB.path_constraints.pose_constraint[0].pose.pose.orientation.y = 0;

      //check that the path obeys constraints
      EXPECT_TRUE(currentStateSatisfiesPathConstraints(km,goalB));
      
    }

    if (elapsed.toSec() > 10.0)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
      EXPECT_TRUE(elapsed.toSec() < 10.0);
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
      EXPECT_TRUE(state == actionlib::SimpleClientGoalState::SUCCEEDED);

      double dist_pose;
      double dist_angle;
      
      diffConfig(km,goalA,dist_pose,dist_angle);

      //close enough - summed tolerances
      EXPECT_TRUE(dist_pose < .01+.01+.002);
      EXPECT_TRUE(dist_angle < .05*3);

      EXPECT_TRUE(finalStateMatchesGoal(km,goalB));
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
