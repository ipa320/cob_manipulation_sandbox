/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/* \author: E. Gil Jones */

#ifndef REGRESSION_TEST_ARM_HELPERS
#define REGRESSION_TEST_ARM_HELPERS

#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <planning_environment/util/kinematic_state_constraint_evaluator.h>
#include <move_arm/MoveArmAction.h>

#include <planning_models/kinematic_state.h>

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

tf::Transform effPosition(const planning_environment::KinematicModelStateMonitor &km, const move_arm::MoveArmGoal &goal)
{
    planning_models::KinematicState sp(*km.getRobotState());
    goalToState(goal, sp);
    km.getKinematicModel()->computeTransforms(sp.getParams());
    return km.getKinematicModel()->getJoint("r_wrist_roll_joint")->after->globalTrans;
}

void diffConfig(const planning_environment::KinematicModelStateMonitor &km, move_arm::MoveArmGoal &goal, double& dist, double& angle)
{
  //getting goal pose from km, I hope
  tf::Transform pose1 = effPosition(km, goal);
  
  //getting current pose from km
  std::vector<std::string> names(7);
  names[0] = "r_shoulder_pan_joint";
  names[1] = "r_shoulder_lift_joint";
  names[2] = "r_upper_arm_roll_joint";
  names[3] = "r_elbow_flex_joint";
  names[4] = "r_forearm_roll_joint";
  names[5] = "r_wrist_flex_joint";
  names[6] = "r_wrist_roll_joint";
  move_arm::MoveArmGoal temp;
  setupGoal(names,temp);

  planning_models::KinematicState sp(*(km.getRobotState()));
  sp.enforceBounds();
  for (unsigned int i = 0 ; i < names.size() ; ++i)
  {
    temp.goal_constraints.joint_constraint[i].value[0] =
      sp.getParamsJoint(names[i])[0];
  }

  tf::Transform pose2 = effPosition(km, temp);
  dist = pose1.getOrigin().distance(pose2.getOrigin());
  angle = pose1.getRotation().angle(pose2.getRotation());
}

// bool checkCurrentStateVersusEffPoseConstraint(const planning_environment::KinematicModelStateMonitor &km, const arm_navigation_msgs::PoseConstraint& constraints) {
  
//   for (unsigned int i = 0 ; i < constraints.joint_constraint.size() ; ++i)
//   {
//     sp.setParamsJoint(&constraints.joint_constraint[i].value[0],
//                       constraints.joint_constraint[i].joint_name);
//   }
 
//   km.getKinematicModel()->computeTransforms(sp.getParams());
//   tf::Transform pose1 = km.getKinematicModel()->getJoint("r_wrist_roll_joint")->after->globalTrans;
  
//   planning_models::KinematicState sp(*km.getRobotState());
//   move_arm::MoveArmGoal temp;
//   setupGoal(names,temp);

//   planning_models::KinematicState sp(*(km.getRobotState()));
//   sp.enforceBounds();
//   for (unsigned int i = 0 ; i < names.size() ; ++i)
//   {
//     temp.goal_constraints.joint_constraint[i].value[0] =
//       sp.getParamsJoint(names[i])[0];
//   }

//   tf::Transform pose2 = effPosition(km, temp);
  
//   tf::Vector3 origin1 = pose1.getOrigin();
//   tf::Vector3 origin2 = pose2.getOrigin();
// }

bool finalStateMatchesGoal(const planning_environment::KinematicModelStateMonitor &km, move_arm::MoveArmGoal &goal) 
{
  planning_models::KinematicState sp(*km.getRobotState());

  km.getKinematicModel()->computeTransforms(sp.getParams());

  planning_environment::KinematicConstraintEvaluatorSet ks;
  ks.add(km.getKinematicModel(),goal.goal_constraints.joint_constraint);
  ks.add(km.getKinematicModel(),goal.goal_constraints.pose_constraint);

  ks.print(std::cout);
  
  return ks.decide(sp.getParams(),true);
  
}

bool currentStateSatisfiesPathConstraints(const planning_environment::KinematicModelStateMonitor &km, move_arm::MoveArmGoal &goal) {

  planning_models::KinematicState sp(*km.getRobotState());

  km.getKinematicModel()->computeTransforms(sp.getParams());
 
  planning_environment::KinematicConstraintEvaluatorSet ks;
  ks.add(km.getKinematicModel(),goal.path_constraints.joint_constraint);
  ks.add(km.getKinematicModel(),goal.path_constraints.pose_constraint);

  ks.print(std::cout);

  bool ok = ks.decide(sp.getParams(),true);
  
  if(!ok) {
    //to get a print out
    ks.decide(sp.getParams(),true);
  }

  return ok;
}

#endif
