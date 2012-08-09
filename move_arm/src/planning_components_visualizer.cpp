/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace arm_navigation_msgs;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace planning_environment;
using namespace planning_models;
using namespace geometry_msgs;

static const string VIS_TOPIC_NAME = "planning_components_visualizer";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;

static const ros::Duration PLANNING_DURATION = ros::Duration(5.0);

static const double BASE_TRANS_SPEED = .3;
static const double BASE_ROT_SPEED = .15;

static const double HAND_TRANS_SPEED = .05;
static const double HAND_ROT_SPEED = .15;

static const string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const string PLANNER_SERVICE_NAME = "/ompl_planning/plan_kinematic_path";
static const string TRAJECTORY_FILTER_SERVICE_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";

typedef map<MenuHandler::EntryHandle, string> MenuEntryMap;
typedef map<string, MenuEntryMap> MenuMap;
typedef map<string, MenuHandler> MenuHandlerMap;


class PlanningComponentsVisualizer
{

  public:

    /// Used to decide which kinematic state the user is controlling. The planner plans from start to end.
    enum IKControlType
    {
      StartPosition, EndPosition
    };

    /// May be for IK control, joint control, or collision objects.
    enum InteractiveMarkerType
    {
      EndEffectorControl, JointControl, CollisionObject
    };

    /// Contains data for selectable markers. Stored in a large map.
    struct SelectableMarker
    {
        /// IK control, joint control, or collision object.
        InteractiveMarkerType type_;

        /// Name of the menu marker.
        string name_;

        /// Name of the 6DOF marker
        string controlName_;

        /// Text above the control.
        string controlDescription_;
    };

    struct StateTrajectoryDisplay
    {

        StateTrajectoryDisplay()
        {
          state_ = NULL;
          reset();
        }

        void reset()
        {
          if(state_ != NULL)
          {
            delete state_;
            state_ = NULL;
          }
          has_joint_trajectory_ = false;
          play_joint_trajectory_ = false;
          show_joint_trajectory_ = false;
          current_trajectory_point_ = 0;
          trajectory_bad_point_ = 0;
          trajectory_error_code_.val = 0;
        }

        KinematicState* state_;
        trajectory_msgs::JointTrajectory joint_trajectory_;unsigned int current_trajectory_point_;
        std_msgs::ColorRGBA color_;
        bool has_joint_trajectory_;
        bool play_joint_trajectory_;
        bool show_joint_trajectory_;
        ArmNavigationErrorCodes trajectory_error_code_;unsigned int trajectory_bad_point_;
    };

    struct GroupCollection
    {
        GroupCollection()
        {
          start_state_ = NULL;
          end_state_ = NULL;
          good_ik_solution_ = false;

          state_trajectory_display_map_["planner"].color_.a = .6;
          state_trajectory_display_map_["planner"].color_.r = 1.0;
          state_trajectory_display_map_["planner"].color_.g = 1.0;
          state_trajectory_display_map_["planner"].color_.b = 0.5;

          state_trajectory_display_map_["filter"].color_.a = .6;
          state_trajectory_display_map_["filter"].color_.r = 0.5;
          state_trajectory_display_map_["filter"].color_.g = 1.0;
          state_trajectory_display_map_["filter"].color_.b = 1.0;
        }

        ~GroupCollection()
        {
          reset();
        }

        void setState(IKControlType type, KinematicState* state)
        {
          switch (type)
          {
            case PlanningComponentsVisualizer::StartPosition:
              if(start_state_ != NULL)
              {
                delete start_state_;
                start_state_ = NULL;
              }
              start_state_ = state;
              break;
            case PlanningComponentsVisualizer::EndPosition:
              if(end_state_ != NULL)
              {
                delete end_state_;
                end_state_ = NULL;
              }
              end_state_ = state;
              break;
          }
        }

        KinematicState* getState(IKControlType type)
        {
          switch (type)
          {
            case PlanningComponentsVisualizer::StartPosition:
              return start_state_;
            case PlanningComponentsVisualizer::EndPosition:
              return end_state_;
          }

          return NULL;
        }

        void reset()
        {
          for(map<string, StateTrajectoryDisplay>::iterator it = state_trajectory_display_map_.begin(); it
              != state_trajectory_display_map_.end(); it++)
          {
            it->second.reset();
          }

          if(start_state_ != NULL)
          {
            delete start_state_;
            start_state_ = NULL;
          }
          if(end_state_ != NULL)
          {
            delete end_state_;
            end_state_ = NULL;
          }

        }

        string name_;
        string ik_link_name_;
        ros::ServiceClient coll_aware_ik_service_;
        ros::ServiceClient non_coll_aware_ik_service_;
        bool good_ik_solution_;
        KinematicState* start_state_;
        KinematicState* end_state_;
        map<string, StateTrajectoryDisplay> state_trajectory_display_map_;
        vector<string> joint_names_;
        tf::Transform last_good_state_;
    };

    PlanningComponentsVisualizer()
    {
      ik_control_type_ = EndPosition;
      num_collision_poles_ = 0;
      collision_aware_ = true;
      cm_ = new CollisionModels("robot_description");
      vis_marker_publisher_ = nh_.advertise<Marker> (VIS_TOPIC_NAME, 128);
      vis_marker_array_publisher_ = nh_.advertise<MarkerArray> (VIS_TOPIC_NAME + "_array", 128);
      joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState> ("joint_states", 10);
      constrain_rp_ = false;

      process_function_ptr_ = boost::bind(&PlanningComponentsVisualizer::processInteractiveFeedback, this, _1);

      while(!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(1.0)))
      {
        ROS_INFO_STREAM("Waiting for planning scene service " << SET_PLANNING_SCENE_DIFF_NAME);
      }

      set_planning_scene_diff_client_
          = nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_NAME);

      while(!ros::service::waitForService(PLANNER_SERVICE_NAME, ros::Duration(1.0)))
      {
        ROS_INFO_STREAM("Waiting for planner service " << PLANNER_SERVICE_NAME);
      }

      planner_service_client_ = nh_.serviceClient<GetMotionPlan> (PLANNER_SERVICE_NAME, true);

      while(!ros::service::waitForService(TRAJECTORY_FILTER_SERVICE_NAME, ros::Duration(1.0)))
      {
        ROS_INFO_STREAM("Waiting for trajectory filter service " << TRAJECTORY_FILTER_SERVICE_NAME);
      }

      trajectory_filter_service_client_
          = nh_.serviceClient<FilterJointTrajectoryWithConstraints> (TRAJECTORY_FILTER_SERVICE_NAME, true);

      const map<string, KinematicModel::GroupConfig>& group_config_map =
          cm_->getKinematicModel()->getJointModelGroupConfigMap();

      for(map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin(); it
          != group_config_map.end(); it++)
      {
        if(!it->second.base_link_.empty())
        {
          group_map_[it->first].name_ = it->first;
          group_map_[it->first].ik_link_name_ = it->second.tip_link_;
          string ik_service_name = cm_->getKinematicModel()->getRobotName() + "_" + it->first + "_kinematics/";
          string coll_aware_name = ik_service_name + "get_constraint_aware_ik";
          string non_coll_aware_name = ik_service_name + "get_ik";

          while(!ros::service::waitForService(coll_aware_name, ros::Duration(1.0)))
          {
            ROS_INFO_STREAM("Waiting for service " << coll_aware_name);
          }

          while(!ros::service::waitForService(non_coll_aware_name, ros::Duration(1.0)))
          {
            ROS_INFO_STREAM("Waiting for service " << non_coll_aware_name);
          }

          group_map_[it->first].coll_aware_ik_service_ = nh_.serviceClient<
              kinematics_msgs::GetConstraintAwarePositionIK> (coll_aware_name, true);

          group_map_[it->first].non_coll_aware_ik_service_
              = nh_.serviceClient<kinematics_msgs::GetPositionIK> (non_coll_aware_name, true);
        }
      }
      robot_state_ = new KinematicState(cm_->getKinematicModel());
      robot_state_->setKinematicStateToDefault();
      sendPlanningScene();

      // Create a new interactive marker server.
      interactive_marker_server_.reset(new InteractiveMarkerServer("planning_visualizer_controls", "", false));

      // Allocate memory to each of the menu entry maps.
      menu_entry_maps_["End Effector"] = MenuEntryMap();
      menu_entry_maps_["End Effector Selection"] = MenuEntryMap();
      menu_entry_maps_["Top Level"] = MenuEntryMap();
      menu_entry_maps_["Collision Object"] = MenuEntryMap();
      menu_entry_maps_["Collision Object Selection"] = MenuEntryMap();

      // Allocate memory to the menu handlers
      menu_handler_map_["End Effector"];
      menu_handler_map_["End Effector Selection"];
      menu_handler_map_["Top Level"];
      menu_handler_map_["Collision Object"];
      menu_handler_map_["Collision Object Selection"];

      // Create end effector menus
      start_position_handle_ = registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"],
                                                 "Set Start Position");
      end_position_handle_ = registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"],
                                               "Set End Position");
      constrain_rp_handle_ = registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"],
                                                "Constrain in Roll and Pitch");
                                                
      menu_handler_map_["End Effector"].setCheckState(start_position_handle_, MenuHandler::UNCHECKED);
      menu_handler_map_["End Effector"].setCheckState(end_position_handle_, MenuHandler::CHECKED);
      menu_handler_map_["End Effector"].setCheckState(constrain_rp_handle_, MenuHandler::UNCHECKED);

      registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"], "Plan");
      registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"], "Filter Trajectory");
      registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"], "Randomly Perturb");
      registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"], "Go To Last Good State");
      registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"], "Deselect");
      registerMenuEntry(menu_handler_map_["End Effector Selection"], menu_entry_maps_["End Effector Selection"],
                        "Select");


      // Create collision object menus
      registerMenuEntry(menu_handler_map_["Collision Object Selection"],
                        menu_entry_maps_["Collision Object Selection"], "Select");
      registerMenuEntry(menu_handler_map_["Collision Object Selection"],
                        menu_entry_maps_["Collision Object Selection"], "Delete");
      registerMenuEntry(menu_handler_map_["Collision Object"], menu_entry_maps_["Collision Object"], "Deselect");
      registerMenuEntry(menu_handler_map_["Collision Object"], menu_entry_maps_["Collision Object"], "Delete");

      // Create top level menu.
      registerMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"], "Create Pole");
      ik_control_handle_ = registerMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"],
                                             "IK Control");
      joint_control_handle_ = registerMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"],
                                                "Joint Control");
      collision_aware_handle_ = registerMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"],
                                                  "Collision Aware");
      menu_handler_map_["Top Level"].setCheckState(ik_control_handle_, MenuHandler::CHECKED);
      menu_handler_map_["Top Level"].setCheckState(joint_control_handle_, MenuHandler::UNCHECKED);
      menu_handler_map_["Top Level"].setCheckState(collision_aware_handle_, MenuHandler::CHECKED);

      is_ik_control_active_ = true;
      is_joint_control_active_ = false;

      MenuHandler::EntryHandle sub_menu_handle = menu_handler_map_["Top Level"].insert("Select Planning Chain");

      // "Select Planning Chain" sub menu.
      unsigned int cmd = 0;
      for(map<string, GroupCollection>::iterator it = group_map_.begin(); it != group_map_.end(); it++)
      {
        registerSubMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"], it->first, sub_menu_handle);

        // These positions will be reset by main()
        makeSelectableMarker(PlanningComponentsVisualizer::EndEffectorControl,
                             tf::Transform(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f), tf::Vector3(0.0f, 0.0f, 0.0f)), it->first,
                             it->first, 0.5f);
        cmd++;
      }
      // Create menu marker.
      makeTopLevelMenu();

      interactive_marker_server_->applyChanges();

      ROS_INFO_STREAM("Initialized");

    }

    ~PlanningComponentsVisualizer()
    {
      deleteKinematicStates();
      delete robot_state_;
      delete cm_;
    }

    /////
    /// @brief Generates a new collision pole ID.
    /// @return an int, which is unique.
    /////
    int nextCollisionPole()
    {
      return ++num_collision_poles_;
    }

    /////
    /// @brief deletes the collision pole with the given ID.
    /// @param num the ID of the pole.
    /////
    void removeCollisionPole(int num)
    {
      stringstream id;
      id << "pole_" << num;
      removeCollisionPoleByName(id.str());
    }

    /////
    /// @brief deletes the collision pole with the given name.
    /// @param id the name of the pole.
    /////
    void removeCollisionPoleByName(string id)
    {
      ROS_INFO("Removing collision pole %s", id.c_str());
      arm_navigation_msgs::CollisionObject& cylinder_object = collision_poles_[id];
      cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
    }

    /////
    /// @brief Creates a collision pole with the given ID at location given by pose. Creates a selectable marker too.
    /// @param num the ID of the pole. (must be unique)
    /// @param pose the location and orientation of the pole.
    /////
    void createCollisionPole(int num, Pose pose)
    {
      ROS_INFO("Creating collision pole %d", num);

      arm_navigation_msgs::CollisionObject cylinder_object;
      cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
      cylinder_object.header.stamp = ros::Time::now();
      cylinder_object.header.frame_id = "/" + cm_->getWorldFrameId();
      arm_navigation_msgs::Shape object;
      object.type = arm_navigation_msgs::Shape::CYLINDER;
      object.dimensions.resize(2);
      object.dimensions[0] = .1;
      object.dimensions[1] = 2.0;

      cylinder_object.shapes.push_back(object);
      cylinder_object.poses.push_back(pose);
      stringstream id;
      id << "pole_" << num;
      cylinder_object.id = id.str();
      collision_poles_[id.str()] = cylinder_object;

      tf::Transform cur = toBulletTransform(pose);
      makePoleContextMenu(cur, id.str(), "", 2.0f);
    }

    void sendPlanningScene()
    {
      ROS_INFO("Sending Planning Scene....");

      lock_.lock();
      arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
      arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

      vector<string> removals;
      // Handle additions and removals of planning scene objects.
      for(map<string, arm_navigation_msgs::CollisionObject>::const_iterator it = collision_poles_.begin(); it
          != collision_poles_.end(); it++)
      {
        string name = it->first;
        arm_navigation_msgs::CollisionObject object = it->second;

        // Add or remove objects.
        if(object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::REMOVE)
        {
          ROS_INFO("Adding Collision Pole %s", object.id.c_str());
          planning_scene_req.planning_scene_diff.collision_objects.push_back(object);
        }
        else
        {
          removals.push_back(it->first);
        }
      }

      // Delete collision poles from the map which were removed.
      for(size_t i = 0; i < removals.size(); i++)
      {
        collision_poles_.erase(removals[i]);
      }

      convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                        planning_scene_req.planning_scene_diff.robot_state);


      KinematicState* startState = NULL;
      KinematicState* endState = NULL;
      map<string, double> startStateValues;
      map<string, double> endStateValues;

      if(current_group_name_ != "")
      {
        startState = group_map_[current_group_name_].getState(StartPosition);
        endState = group_map_[current_group_name_].getState(EndPosition);

        if(startState != NULL)
        {
          startState->getKinematicStateValues(startStateValues);
        }
        if(endState != NULL)
        {
          endState->getKinematicStateValues(endStateValues);
        }
      }

      deleteKinematicStates();


      if(robot_state_ != NULL)
      {
        ROS_INFO("Reverting planning scene to default.");
        cm_->revertPlanningScene(robot_state_);
        robot_state_ = NULL;
      }

      if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res))
      {
        ROS_WARN("Can't get planning scene");
        lock_.unlock();
        return;
      }

      robot_state_ = cm_->setPlanningScene(planning_scene_res.planning_scene);

      if(robot_state_ == NULL)
      {
        ROS_ERROR("Something wrong with planning scene");
        lock_.unlock();
        return;
      }

      if(current_group_name_ != "")
      {
        ROS_INFO("Resetting state...");
        group_map_[current_group_name_].setState(StartPosition, new KinematicState(robot_state_->getKinematicModel()));
        group_map_[current_group_name_].setState(EndPosition, new KinematicState(robot_state_->getKinematicModel()));
        startState = group_map_[current_group_name_].getState(StartPosition);
        endState = group_map_[current_group_name_].getState(EndPosition);

        if(startState != NULL)
        {          
          ROS_INFO("Resetting start state.");
          startState->setKinematicState(startStateValues);
        }

        if(endState != NULL)
        {
          ROS_INFO("Resetting end state.");
          endState->setKinematicState(endStateValues);
        }
      }
      lock_.unlock();
      ROS_INFO("Planning scene sent.");
    }

    void selectPlanningGroup(unsigned int entry)
    {
      ROS_INFO("Selecting planning group %u", entry);
      lock_.lock();
      vector<string> names;
      for(map<string, GroupCollection>::iterator it = group_map_.begin(); it != group_map_.end(); it++)
      {
        names.push_back(it->first);
      }
      string old_group_name = current_group_name_;

      current_group_name_ = names[entry];

      if(group_map_[current_group_name_].end_state_ != NULL)
      {
        ROS_WARN_STREAM("Selecting with non NULL");
        // group_map_[current_group_name_].setState(EndPosition,
        //                                          new KinematicState(*group_map_[current_group_name_].end_state_));
      }
      else
      {
        group_map_[current_group_name_].setState(EndPosition, new KinematicState(*robot_state_));
      }

      if(group_map_[current_group_name_].start_state_ != NULL)
      {
        ROS_WARN_STREAM("Selecting with non NULL");
        // group_map_[current_group_name_].setState(EndPosition,
        //                                          new KinematicState(*group_map_[current_group_name_].start_state_));
      }
      else
      {
        group_map_[current_group_name_].setState(StartPosition, new KinematicState(*robot_state_));
      }

      moveEndEffectorMarkers(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

      // If we previously selected a planning group, deselect its marker.
      if(old_group_name != "" && selectableMarkerExists(old_group_name + "_selectable"))
      {
        GroupCollection& gc = group_map_[old_group_name];
        tf::Transform cur = robot_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
        deselectMarker(selectable_markers_[old_group_name + "_selectable"], cur);

        if(is_joint_control_active_)
        {
          deleteJointMarkers(group_map_[old_group_name]);
        }
      }

      // Select the new planning group's marker.
      if(is_ik_control_active_ && selectableMarkerExists(current_group_name_ + "_selectable"))
      {
        GroupCollection& gc = group_map_[current_group_name_];
        tf::Transform cur = gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
        selectMarker(selectable_markers_[current_group_name_ + "_selectable"], cur);
        createSelectableJointMarkers(gc);
      }
      else if(is_joint_control_active_)
      {
        GroupCollection& gc = group_map_[current_group_name_];
        createSelectableJointMarkers(gc);
      }
      interactive_marker_server_->erase(current_group_name_ + "_selectable");
      interactive_marker_server_->applyChanges();
      lock_.unlock();
      ROS_INFO("Planning group selected.");
    }

    bool isValidJointName(GroupCollection& gc, string name)
    {
      for(size_t i = 0; i < gc.joint_names_.size(); i++)
      {
        if(name == gc.joint_names_[i])
        {
          return true;
        }
      }
      return false;
    }

    void moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw,
                                bool coll_aware = true)

    {
      lock_.lock();
      GroupCollection& gc = group_map_[current_group_name_];
      tf::Transform cur = gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
      double mult = CONTROL_SPEED / 100.0;

      tf::Vector3& curOrigin = cur.getOrigin();
      tf::Vector3 newOrigin(curOrigin.x() + (vx * mult), curOrigin.y() + (vy * mult), curOrigin.z() + (vz * mult));
      cur.setOrigin(newOrigin);

      tfScalar roll, pitch, yaw;

      cur.getBasis().getRPY(roll, pitch, yaw);
      roll += vr * mult;
      pitch += vp * mult;
      yaw += vw * mult;

      if(roll > 2 * M_PI)
      {
        roll -= 2 * M_PI;
      }
      else if(roll < -2 * M_PI)
      {
        roll += 2 * M_PI;
      }

      if(pitch > 2 * M_PI)
      {
        pitch -= 2 * M_PI;
      }
      else if(pitch < -2 * M_PI)
      {
        pitch += 2 * M_PI;
      }

      cur.getBasis().setRPY(roll, pitch, yaw);

      setNewEndEffectorPosition(gc, cur, coll_aware);

      lock_.unlock();
    }

    void deleteJointMarkers(GroupCollection& gc)
    {
      for(size_t i = 0; i < gc.joint_names_.size(); i++)
      {
        interactive_marker_server_->erase(gc.joint_names_[i] + "_joint_control");
      }
    }

    ////
    /// @brief Given the group collection, creates a set of selectable markers for each joint in the group.
    /// @param gc the group collection to create joint markers for.
    ////
    void createSelectableJointMarkers(GroupCollection& gc)
    {
      if(!is_joint_control_active_)
      {
        return;
      }

      // For each joint model, find the location of its axis and make a control there.
      for(size_t i = 0; i < gc.joint_names_.size(); i++)
      {
        const string& jointName = gc.joint_names_[i];
        KinematicModel::JointModel* model =
            (KinematicModel::JointModel*)(gc.getState(ik_control_type_)->getKinematicModel()->getJointModel(jointName));
        KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<KinematicModel::RevoluteJointModel*> (model);
        KinematicModel::PrismaticJointModel* prismaticJoint =
            dynamic_cast<KinematicModel::PrismaticJointModel*> (model);

        joint_clicked_map_[jointName + "_joint_control"] = false;

        if(model->getParentLinkModel() != NULL)
        {
          string parentLinkName = model->getParentLinkModel()->getName();
          string childLinkName = model->getChildLinkModel()->getName();
          tf::Transform
              transform =
                  gc.getState(ik_control_type_)->getLinkState(parentLinkName)->getGlobalLinkTransform()
                      * (gc.getState(ik_control_type_)->getKinematicModel()->getLinkModel(childLinkName)->getJointOriginTransform()
                          * (gc.getState(ik_control_type_)->getJointState(jointName)->getVariableTransform()));

          joint_prev_transform_map_[jointName + "joint_control"] = transform;

          const shapes::Shape* linkShape = model->getChildLinkModel()->getLinkShape();
          const shapes::Mesh* meshShape = dynamic_cast<const shapes::Mesh*> (linkShape);

          double maxDimension = 0.0f;
          if(meshShape != NULL)
          {
            for(unsigned int i = 0; i < meshShape->vertexCount; i++)
            {
              double x = meshShape->vertices[3 * i];
              double y = meshShape->vertices[3 * i];
              double z = meshShape->vertices[3 * i];

              if(abs(maxDimension) < abs(sqrt(x*x+y*y+z*z)))
              {
                maxDimension = abs(x);
              }

            }

            maxDimension *= 3.0;

            maxDimension = max(0.15, maxDimension);
            maxDimension = min(0.5, maxDimension);
          }
          else
          {
            maxDimension = 0.15;
          }

          if(revoluteJoint != NULL)
          {
            makeInteractive1DOFRotationMarker(transform,
                                              revoluteJoint->axis_,
                                              model->getName() + "_joint_control",
                                              "",
                                              (float)maxDimension,
                                              gc.getState(ik_control_type_)->getJointState(jointName)->getJointStateValues()[0]);
          }
          else if(prismaticJoint != NULL)
          {
            maxDimension *= 3.0;
            makeInteractive1DOFTranslationMarker(transform,
                                                 prismaticJoint->axis_,
                                                 model->getName() + "_joint_control",
                                                 "",
                                                 (float)maxDimension,
                                                 gc.getState(ik_control_type_)-> getJointState(jointName)->getJointStateValues()[0]);
          }

        }
      }
      interactive_marker_server_->applyChanges();
    }

    /////
    /// @brief Sets the kinematic state of a joint in the given group collection.
    /// @param gc the group collection that the joint is in.
    /// @param value, a transform that the joint will attempt to match.
    /////
  void setJointState(GroupCollection& gc, std::string& joint_name, tf::Transform value)
    {


      KinematicState* current_state = gc.getState(ik_control_type_);
      string parent_link =
          gc.getState(ik_control_type_)->getKinematicModel()->getJointModel(joint_name)->getParentLinkModel()->getName();
      string child_link =
          gc.getState(ik_control_type_)->getKinematicModel()->getJointModel(joint_name)->getChildLinkModel()->getName();
      KinematicState::JointState* joint_state = current_state->getJointState(joint_name);
      const KinematicModel::JointModel* joint_model = joint_state->getJointModel();

      const KinematicModel::RevoluteJointModel* rev_model = dynamic_cast<const KinematicModel::RevoluteJointModel*>(joint_model);

      bool is_rotational = (rev_model != NULL);
      bool is_continuous = is_rotational && rev_model->continuous_;
      bool is_prismatic = (dynamic_cast<const KinematicModel::PrismaticJointModel*>(joint_model) != NULL);

      KinematicState::LinkState* link_state = current_state->getLinkState(parent_link);
      tf::Transform transformed_value;

      if(is_prismatic)
      {
        value.setRotation(joint_state->getVariableTransform().getRotation());
        transformed_value = current_state->getLinkState(child_link)->getLinkModel()->getJointOriginTransform().inverse() * link_state->getGlobalLinkTransform().inverse() * value;
      }
      else if(is_rotational)
      {
        transformed_value = current_state->getLinkState(child_link)->getLinkModel()->getJointOriginTransform().inverse() * link_state->getGlobalLinkTransform().inverse() * value;
      }
      double old_actual_value = joint_state->getJointStateValues()[0];
      double old_value;
      if(prev_joint_control_value_map_.find(joint_name) == prev_joint_control_value_map_.end()) {
        old_value = old_actual_value;
      } else {
        old_value = prev_joint_control_value_map_[joint_name];
      }

      joint_state->setJointStateValues(transformed_value);
      double new_value = joint_state->getJointStateValues()[0];
      prev_joint_control_value_map_[joint_name] = new_value;

      double low_bound, high_bound;
      joint_state->getJointValueBounds(joint_state->getName(), low_bound, high_bound);

      double apply_diff;

      if(is_rotational) {
        //more neg
        if(old_value < -M_PI/2.0 && new_value > M_PI/2.0) {
          apply_diff = -((-M_PI-old_value)+(M_PI-new_value));
        } else if(old_value > M_PI/2.0 && new_value < -M_PI/2.0) {
          apply_diff = (M_PI-old_value)+(-M_PI-new_value);
        } else {
          apply_diff = new_value-old_value;
        }
        ROS_DEBUG_STREAM("Old value " << old_value << " new value " << new_value << " apply diff " << apply_diff);
        double apply_value;
        if(!is_continuous) {
          if(apply_diff > 0.0) {
            apply_value = fmin(old_actual_value+apply_diff, high_bound);
          } else {
            apply_value = fmax(old_actual_value+apply_diff, low_bound);
          }
        } else {
          apply_value = old_actual_value+apply_diff;
        }
        std::vector<double> val_vec(1, apply_value);
        joint_state->setJointStateValues(val_vec);
      } else {
        std::vector<double> val_vec;
        if(!current_state->isJointWithinBounds(joint_name))
        {
          if(new_value < low_bound) {
            val_vec.push_back(low_bound);
          } else if(new_value > high_bound) {
            val_vec.push_back(high_bound);
          } else {
            val_vec.push_back(old_value);
            ROS_INFO_STREAM("Weird bounds behavior " << new_value << " low " << low_bound << " high " << high_bound);
          }
          ROS_DEBUG_STREAM("New val " << new_value << " low bound " << low_bound << " high bound " << high_bound << " res " << val_vec[0]);
          joint_state->setJointStateValues(val_vec);
        }
      }
      map<string, double> state_map;
      current_state->getKinematicStateValues(state_map);
      current_state->setKinematicState(state_map);
      robot_state_->setKinematicState(state_map);
      // Send state to robot model.
      updateJointStates(gc);

      createSelectableJointMarkers(gc);
      
      if(is_ik_control_active_)
      {
        selectMarker(selectable_markers_[current_group_name_ + "_selectable"],
                     current_state->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
      }
    }

    void resetToLastGoodState(GroupCollection& gc)
    {
      setNewEndEffectorPosition(gc, gc.last_good_state_, collision_aware_);
      if(is_ik_control_active_)
      {
        selectMarker(selectable_markers_[current_group_name_ + "_selectable"],
                     gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
      }
    }

    void setNewEndEffectorPosition(GroupCollection& gc, tf::Transform& cur, bool coll_aware)
    {
      if(!gc.getState(ik_control_type_)->updateKinematicStateWithLinkAt(gc.ik_link_name_, cur))
      {
        ROS_INFO("Problem");
      }

      if(solveIKForEndEffectorPose(gc, coll_aware, constrain_rp_))
      {
        gc.good_ik_solution_ = true;
        gc.last_good_state_ = cur;
      }
      else
      {
        gc.good_ik_solution_ = false;
      }
    }

  void determinePitchRollConstraintsGivenState(const PlanningComponentsVisualizer::GroupCollection& gc,
                                               const planning_models::KinematicState& state,
                                               arm_navigation_msgs::OrientationConstraint& goal_constraint,
                                               arm_navigation_msgs::OrientationConstraint& path_constraint) const
  {
    tf::Transform cur = state.getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
    //tfScalar roll, pitch, yaw;
    //cur.getBasis().getRPY(roll,pitch,yaw);
    goal_constraint.header.frame_id = cm_->getWorldFrameId();
    goal_constraint.header.stamp = ros::Time::now();
    goal_constraint.link_name = gc.ik_link_name_;
    tf::quaternionTFToMsg(cur.getRotation(), goal_constraint.orientation);
    goal_constraint.absolute_roll_tolerance = 0.04;
    goal_constraint.absolute_pitch_tolerance = 0.04;
    goal_constraint.absolute_yaw_tolerance = M_PI;
    path_constraint.header.frame_id = cm_->getWorldFrameId();
    path_constraint.header.stamp = ros::Time::now();
    path_constraint.link_name = gc.ik_link_name_;
    tf::quaternionTFToMsg(cur.getRotation(), path_constraint.orientation);
    path_constraint.type = path_constraint.HEADER_FRAME;
    path_constraint.absolute_roll_tolerance = 0.1;
    path_constraint.absolute_pitch_tolerance = 0.1;
    path_constraint.absolute_yaw_tolerance = M_PI;
  }


  bool solveIKForEndEffectorPose(PlanningComponentsVisualizer::GroupCollection& gc, bool coll_aware = true,
                                 bool constrain_pitch_and_roll = false, double change_redundancy = 0.0)
    {
      kinematics_msgs::PositionIKRequest ik_request;
      ik_request.ik_link_name = gc.ik_link_name_;
      ik_request.pose_stamped.header.frame_id =  cm_->getWorldFrameId();
      ik_request.pose_stamped.header.stamp = ros::Time::now();
      tf::poseTFToMsg(gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(),
                      ik_request.pose_stamped.pose);
      convertKinematicStateToRobotState(*gc.getState(ik_control_type_), ros::Time::now(), cm_->getWorldFrameId(),
                                        ik_request.robot_state);
      ik_request.ik_seed_state = ik_request.robot_state;

      // if(change_redundancy != 0.0) {
      //   for(unsigned int i = 0; i < ik_request.ik_seed_state.joint_state.name.size(); i++) {
      //     if(ik_request.ik_seed_state.joint_state.name[i] == redundancy_joint_) {
      //       ik_request.ik_seed_state.joint_state.position[i] += change_redundancy;
      //     }
      //   }
      // }
      map<string, double> joint_values;
      vector<string> joint_names;

      if(coll_aware)
      {
        kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
        kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
        if(constrain_pitch_and_roll) {
          IKControlType other_state;
          if(ik_control_type_ == EndPosition)
          {
            other_state = StartPosition;
          }
          else
          {
            other_state = EndPosition;
          }
          arm_navigation_msgs::Constraints goal_constraints;
          goal_constraints.orientation_constraints.resize(1);
          arm_navigation_msgs::Constraints path_constraints;
          path_constraints.orientation_constraints.resize(1);
          determinePitchRollConstraintsGivenState(gc,
                                                  *gc.getState(other_state),
                                                  goal_constraints.orientation_constraints[0],
                                                  path_constraints.orientation_constraints[0]);
          arm_navigation_msgs::ArmNavigationErrorCodes err;
          if(!cm_->isKinematicStateValid(*gc.getState(ik_control_type_),
                                         std::vector<std::string>(),
                                         err,
                                         goal_constraints,
                                         path_constraints)) {
            ROS_INFO_STREAM("Violates rp constraints");
            return false;
          }
          ik_req.constraints = goal_constraints;
        }
        ik_req.ik_request = ik_request;
        ik_req.timeout = ros::Duration(0.2);
        if(!gc.coll_aware_ik_service_.call(ik_req, ik_res))
        {
          ROS_INFO("Problem with ik service call");
          return false;
        }
        if(ik_res.error_code.val != ik_res.error_code.SUCCESS)
        {
          ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
          return false;
        }
        joint_names = ik_res.solution.joint_state.name;
        gc.joint_names_.clear();
        gc.joint_names_ = joint_names;
        for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
        {
          joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
        }

      }
      else
      {
        kinematics_msgs::GetPositionIK::Request ik_req;
        kinematics_msgs::GetPositionIK::Response ik_res;
        ik_req.ik_request = ik_request;
        ik_req.timeout = ros::Duration(0.2);
        if(!gc.non_coll_aware_ik_service_.call(ik_req, ik_res))
        {
          ROS_INFO("Problem with ik service call");
          return false;
        }
        if(ik_res.error_code.val != ik_res.error_code.SUCCESS)
        {
          ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
          return false;
        }
        for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
        {
          joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
        }

      }

      lock_.lock();
      gc.getState(ik_control_type_)->setKinematicState(joint_values);
      lock_.unlock();

      createSelectableJointMarkers(gc);
      if(coll_aware)
      {
        Constraints emp_con;
        ArmNavigationErrorCodes error_code;

        if(!cm_->isKinematicStateValid(*gc.getState(ik_control_type_), joint_names, error_code, emp_con, emp_con, true))
        {
          ROS_INFO_STREAM("Problem with response");
        }
      }

      updateJointStates(gc);

      return true;
    }


    /////
    /// @brief Sends the joint states of the given group collection to the robot state publisher.
    /// @param gc the group collection to publish the states for.
    /////
    void updateJointStates(GroupCollection& gc)
    {
      sensor_msgs::JointState msg;
      msg.header.frame_id =  cm_->getWorldFrameId();
      msg.header.stamp = ros::Time::now();

      vector<KinematicState::JointState*> jointStates = gc.getState(ik_control_type_)->getJointStateVector();

      map<string, double> stateMap;
      gc.getState(ik_control_type_)->getKinematicStateValues(stateMap);
      robot_state_->setKinematicState(stateMap);

      for(size_t i = 0; i < jointStates.size(); i++)
      {
        KinematicState::JointState* state = jointStates[i];
        msg.name.push_back(state->getName());

        // Assume that joints only have one value.
        if(state->getJointStateValues().size() > 0)
        {
          msg.position.push_back(state->getJointStateValues()[0]);
        }
        else
        {
          msg.position.push_back(0.0f);
        }
      }
      joint_state_lock_.lock();
      last_joint_state_msg_ = msg;
      joint_state_lock_.unlock();

    }
  void publishJointStates() {
    joint_state_lock_.lock();
    last_joint_state_msg_.header.frame_id =  cm_->getWorldFrameId();
    last_joint_state_msg_.header.stamp = ros::Time::now();
    joint_state_publisher_.publish(last_joint_state_msg_);
    joint_state_lock_.unlock();

    if(cm_->getWorldFrameId() != cm_->getRobotFrameId()) {
      TransformStamped trans;
      trans.header.frame_id = cm_->getWorldFrameId();
      trans.header.stamp = ros::Time::now();
      trans.child_frame_id = cm_->getRobotFrameId();
      trans.transform.rotation.w = 1.0;
      transform_broadcaster_.sendTransform(trans);
    }
  }

    bool planToEndEffectorState(PlanningComponentsVisualizer::GroupCollection& gc)
    {
      MotionPlanRequest motion_plan_request;
      motion_plan_request.group_name = gc.name_;
      motion_plan_request.num_planning_attempts = 1;
      motion_plan_request.allowed_planning_time = PLANNING_DURATION;
      if(!constrain_rp_) {
        const KinematicState::JointStateGroup* jsg = gc.getState(EndPosition)->getJointStateGroup(gc.name_);
        motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());
        vector<double> joint_values;
        jsg->getKinematicStateValues(joint_values);
        for(unsigned int i = 0; i < jsg->getJointNames().size(); i++)
        {
          motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
          motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
          motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01;
          motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01;
        }
      } else {
        motion_plan_request.group_name += "_cartesian";
        motion_plan_request.goal_constraints.position_constraints.resize(1);
        motion_plan_request.goal_constraints.orientation_constraints.resize(1);    
        geometry_msgs::PoseStamped end_effector_wrist_pose;
        tf::poseTFToMsg(gc.getState(EndPosition)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(),
                        end_effector_wrist_pose.pose);
        end_effector_wrist_pose.header.frame_id = cm_->getWorldFrameId();
        arm_navigation_msgs::poseStampedToPositionOrientationConstraints(end_effector_wrist_pose,
                                                                          gc.ik_link_name_,
                                                                          motion_plan_request.goal_constraints.position_constraints[0],
                                                                          motion_plan_request.goal_constraints.orientation_constraints[0]);
        motion_plan_request.path_constraints.orientation_constraints.resize(1);
        determinePitchRollConstraintsGivenState(gc,
                                                *gc.getState(StartPosition),
                                                motion_plan_request.goal_constraints.orientation_constraints[0],
                                                motion_plan_request.path_constraints.orientation_constraints[0]);
      }
      convertKinematicStateToRobotState(*gc.getState(StartPosition), ros::Time::now(), cm_->getWorldFrameId(),
                                        motion_plan_request.start_state);
      GetMotionPlan::Request plan_req;
      plan_req.motion_plan_request = motion_plan_request;
      GetMotionPlan::Response plan_res;
      if(!planner_service_client_.call(plan_req, plan_res))
      {
        ROS_INFO("Something wrong with planner client");
        return false;
      }

      if(gc.state_trajectory_display_map_.find("planner") != gc.state_trajectory_display_map_.end())
      {
        StateTrajectoryDisplay& disp = gc.state_trajectory_display_map_["planner"];
        if(plan_res.error_code.val != plan_res.error_code.SUCCESS)
        {
          disp.trajectory_error_code_ = plan_res.error_code;
          ROS_INFO_STREAM("Bad planning error code " << plan_res.error_code.val);
          gc.state_trajectory_display_map_["planner"].reset();
          return false;
        }
        last_motion_plan_request_ = motion_plan_request;
        playTrajectory(gc, "planner", plan_res.trajectory.joint_trajectory);
        return true;
      }
      else
      {
        return false;
      }
    }

    void randomlyPerturb(PlanningComponentsVisualizer::GroupCollection& gc)
    {
      tf::Transform currentPose = gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();

      int maxTries = 10;
      int numTries = 0;
      bool found = false;
      double posVariance = 0.5;
      double angleVariance = 0.5;

      while(!found && numTries < maxTries)
      {

        double xVar = posVariance*((double)random()/(double)RAND_MAX) - posVariance/2.0;
        double yVar = posVariance*((double)random()/(double)RAND_MAX) - posVariance/2.0;
        double zVar = posVariance*((double)random()/(double)RAND_MAX) - posVariance/2.0;

        double xAngleVar = angleVariance*((double)random()/(double)RAND_MAX) - angleVariance/2.0;
        double yAngleVar = angleVariance*((double)random()/(double)RAND_MAX) - angleVariance/2.0;
        double zAngleVar = angleVariance*((double)random()/(double)RAND_MAX) - angleVariance/2.0;

        double x = currentPose.getOrigin().x() + xVar;
        double y = currentPose.getOrigin().y() + yVar;
        double z = currentPose.getOrigin().z() + zVar;

        double xA = currentPose.getRotation().x() + xAngleVar;
        double yA = currentPose.getRotation().y() + yAngleVar;
        double zA = currentPose.getRotation().z() + zAngleVar;

        tf::Vector3 newPos(x,y,z);
        tf::Quaternion newOrient(xA,yA,zA,1.0);
        tf::Transform newTrans(newOrient,newPos);

        setNewEndEffectorPosition(gc, newTrans, collision_aware_);
        if(gc.good_ik_solution_)
        {
          found = true;
          if(is_ik_control_active_)
          {
            selectMarker(selectable_markers_[current_group_name_ + "_selectable"],
                         gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
          }
        }

        numTries ++;
        posVariance *= 1.1;
      }
    }

    bool filterPlannerTrajectory(PlanningComponentsVisualizer::GroupCollection& gc)
    {
      FilterJointTrajectoryWithConstraints::Request filter_req;
      FilterJointTrajectoryWithConstraints::Response filter_res;

      convertKinematicStateToRobotState(*gc.getState(StartPosition), ros::Time::now(), cm_->getWorldFrameId(),
                                        filter_req.start_state);
      StateTrajectoryDisplay& planner_disp = gc.state_trajectory_display_map_["planner"];
      filter_req.trajectory = planner_disp.joint_trajectory_;
      filter_req.group_name = gc.name_;

      filter_req.goal_constraints = last_motion_plan_request_.goal_constraints;
      filter_req.path_constraints = last_motion_plan_request_.path_constraints;
      filter_req.allowed_time = ros::Duration(2.0);

      if(!trajectory_filter_service_client_.call(filter_req, filter_res))
      {
        ROS_INFO("Problem with trajectory filter");
        gc.state_trajectory_display_map_["filter"].reset();
        return false;
      }
      StateTrajectoryDisplay& filter_disp = gc.state_trajectory_display_map_["filter"];
      if(filter_res.error_code.val != filter_res.error_code.SUCCESS)
      {
        filter_disp.trajectory_error_code_ = filter_res.error_code;
        ROS_INFO_STREAM("Bad trajectory_filter error code " << filter_res.error_code.val);
        gc.state_trajectory_display_map_["filter"].reset();
        return false;
      }
      playTrajectory(gc, "filter", filter_res.trajectory);
      return true;
    }

    bool playTrajectory(PlanningComponentsVisualizer::GroupCollection& gc, const string& source_name,
                        const trajectory_msgs::JointTrajectory& traj)
    {
      lock_.lock();
      if(gc.state_trajectory_display_map_.find(source_name) == gc.state_trajectory_display_map_.end())
      {
        ROS_INFO_STREAM("No state display for group " << gc.name_ << " source name " << source_name);
        lock_.unlock();
        return false;
      }
      StateTrajectoryDisplay& disp = gc.state_trajectory_display_map_[source_name];
      disp.reset();
      disp.joint_trajectory_ = traj;
      disp.has_joint_trajectory_ = true;
      disp.show_joint_trajectory_ = true;
      disp.play_joint_trajectory_ = true;
      disp.state_ = new KinematicState(*robot_state_);
      vector<ArmNavigationErrorCodes> trajectory_error_codes;

      cm_->isJointTrajectoryValid(*disp.state_, disp.joint_trajectory_, last_motion_plan_request_.goal_constraints,
                                  last_motion_plan_request_.path_constraints, disp.trajectory_error_code_,
                                  trajectory_error_codes, false);

      if(disp.trajectory_error_code_.val != disp.trajectory_error_code_.SUCCESS)
      {
        disp.trajectory_bad_point_ = trajectory_error_codes.size() - 1;
      }
      else
      {
        disp.trajectory_bad_point_ = -1;
      }

      moveThroughTrajectory(gc, source_name, 0);
      lock_.unlock();
      return true;
    }

    void moveThroughTrajectory(PlanningComponentsVisualizer::GroupCollection& gc, const string& source_name, int step)
    {
      lock_.lock();
      StateTrajectoryDisplay& disp = gc.state_trajectory_display_map_[source_name];
      unsigned int tsize = disp.joint_trajectory_.points.size();
      if(tsize == 0 || disp.state_ == NULL)
      {
        lock_.unlock();
        return;
      }
      if((int)disp.current_trajectory_point_ + step < 0)
      {
        disp.current_trajectory_point_ = 0;
      }
      else
      {
        disp.current_trajectory_point_ = ((int)disp.current_trajectory_point_) + step;
      }
      if(disp.current_trajectory_point_ >= tsize - 1)
      {
        disp.current_trajectory_point_ = tsize - 1;
        disp.play_joint_trajectory_ = false;
        disp.show_joint_trajectory_ = false;
      }
      map<string, double> joint_values;
      for(unsigned int i = 0; i < disp.joint_trajectory_.joint_names.size(); i++)
      {
        joint_values[disp.joint_trajectory_.joint_names[i]]
            = disp.joint_trajectory_.points[disp.current_trajectory_point_].positions[i];
      }
      disp.state_->setKinematicState(joint_values);
      lock_.unlock();
    }

    /////
    /// @brief Creates a new menu entry for the given menu handler, putting its handle in the given map.
    /// @param handler the menu handler to associate this entry with.
    /// @param map the menu entry map to associate the name with a handle.
    /// @param the name which appears in the menu.
    /// @param subMenuHandle a menu handle to an existing menu entry in the menu handler given.
    /// @return the handle that was registered
    /////
    MenuHandler::EntryHandle registerSubMenuEntry(MenuHandler& handler, MenuEntryMap& map, string name,
                                                  MenuHandler::EntryHandle subMenuHandle)
    {
      MenuHandler::EntryHandle toReturn = handler.insert(subMenuHandle, name, process_function_ptr_);
      map[toReturn] = name;
      return toReturn;
    }

    /////
    /// @brief Creates a new menu entry for the given menu handler, putting its handle in the given map.
    /// @param handler the menu handler to associate this entry with.
    /// @param map the menu entry map to associate the name with a handle.
    /// @param the name which appears in the menu.
    /// @return the handle that was registered
    /////
    MenuHandler::EntryHandle registerMenuEntry(MenuHandler& handler, MenuEntryMap& map, string name)
    {
      MenuHandler::EntryHandle toReturn = handler.insert(name, process_function_ptr_);
      map[toReturn] = name;
      return toReturn;
    }

    void sendMarkers()
    {
      lock_.lock();
      MarkerArray arr;

      std_msgs::ColorRGBA stat_color_;
      stat_color_.a = 1.0;
      stat_color_.r = 0.1;
      stat_color_.g = 0.8;
      stat_color_.b = 0.3;

      std_msgs::ColorRGBA attached_color_;
      attached_color_.a = 1.0;
      attached_color_.r = 0.6;
      attached_color_.g = 0.4;
      attached_color_.b = 0.3;

      cm_->getAllCollisionSpaceObjectMarkers(*robot_state_, arr, "", stat_color_, attached_color_, ros::Duration(0.1));



      if(!current_group_name_.empty())
      {
        std_msgs::ColorRGBA group_color;
        group_color.a = 0.3;
        group_color.r = 0.5;
        group_color.g = 0.9;
        group_color.b = 0.5;

        std_msgs::ColorRGBA updated_color;
        updated_color.a = 0.3;
        updated_color.r = 1.0;
        updated_color.g = 0.5;
        updated_color.b = 1.0;

        std_msgs::ColorRGBA bad_color;
        bad_color.a = 0.6;
        bad_color.r = 1.0;
        bad_color.g = 0.0;
        bad_color.b = 0.0;

        GroupCollection& gc = group_map_[current_group_name_];
        const KinematicModel* kinematic_model = cm_->getKinematicModel();

        IKControlType otherState;
        if(ik_control_type_ == EndPosition)
        {
          otherState = StartPosition;
        }
        else
        {
          otherState = EndPosition;
        }

        if(is_ik_control_active_)
        {
          if(gc.getState(otherState) != NULL)
          {
            cm_->getGroupAndUpdatedJointMarkersGivenState(*gc.getState(otherState), arr, current_group_name_, group_color,
                                                        updated_color, ros::Duration(0.1));
          }
          else
          {
            ROS_ERROR("Other state invalid!");
          }
        }

        if(!gc.good_ik_solution_ && gc.getState(ik_control_type_) != NULL)
        {
          vector<string> lnames =
              kinematic_model->getChildLinkModelNames(kinematic_model->getLinkModel(gc.ik_link_name_));

          cm_->getRobotMarkersGivenState(*gc.getState(ik_control_type_), arr, bad_color,
                                         current_group_name_, ros::Duration(0.1), &lnames);
          cm_->getAttachedCollisionObjectMarkers(*gc.getState(ik_control_type_), arr, current_group_name_, bad_color,
                                                 ros::Duration(.2));

        }
        for(map<string, StateTrajectoryDisplay>::iterator it = gc.state_trajectory_display_map_.begin(); it
            != gc.state_trajectory_display_map_.end(); it++)
        {

          if(it->second.play_joint_trajectory_)
          {
            moveThroughTrajectory(gc, it->first, 5);
          }

          if(it->second.show_joint_trajectory_)
          {
            const vector<const KinematicModel::LinkModel*>& updated_links =
                kinematic_model->getModelGroup(gc.name_)->getUpdatedLinkModels();
            vector<string> lnames;
            lnames.resize(updated_links.size());
            for(unsigned int i = 0; i < updated_links.size(); i++)
            {
              lnames[i] = updated_links[i]->getName();
            }
            cm_->getRobotMarkersGivenState(*(it->second.state_), arr, it->second.color_,
                                           it->first + "_trajectory", ros::Duration(0.1), &lnames);

            cm_->getAttachedCollisionObjectMarkers(*(it->second.state_), arr, it->first + "_trajectory",
                                                   it->second.color_, ros::Duration(0.1));
          }
        }
      }
      vis_marker_array_publisher_.publish(arr);
      lock_.unlock();
    }

    void sendTransforms()
    {
      lock_.lock();
      // TODO: Is it necessary to publish interactive markers each update?
      interactive_marker_server_->applyChanges();
      lock_.unlock();

      // TODO: This was replaced by another node, the robot state publisher.
      /*
       lock_.lock();
       ros::WallTime cur_time = ros::WallTime::now();
       rosgraph_msgs::Clock c;
       c.clock.nsec = cur_time.nsec;
       c.clock.sec = cur_time.sec;
       vector<TransformStamped> trans_vector;
       getAllKinematicStateStampedTransforms(*robot_state_, trans_vector, c.clock);
       //transform_broadcaster_.sendTransform(trans_vector);
       lock_.unlock();
       */
    }

    /////
    /// @brief Creates a box shaped marker.
    /// @param msg the interactive marker to associate this box with.
    /// @param alpha the transparency of the marker.
    /// @return the marker (which is a box).
    /////
    Marker makeMarkerBox(InteractiveMarker &msg, float alpha = 1.0f)
    {
      Marker marker;
      marker.type = Marker::CUBE;
      // Scale is arbitrarily 1/4 of the marker's scale.
      marker.scale.x = msg.scale * 0.25;
      marker.scale.y = msg.scale * 0.25;
      marker.scale.z = msg.scale * 0.25;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = alpha;

      return marker;
    }

    /////
    /// @brief Creates a cylinder shaped marker.
    /// @param msg the interactive marker to associate this cylinder with.
    /// @param alpha the transparency of the marker.
    /// @return the marker (which is a cylinder).
    /////
    Marker makeMarkerCylinder(InteractiveMarker &msg, float alpha = 1.0f)
    {
      Marker marker;
      marker.type = Marker::CYLINDER;
      // Scale is arbitrary
      marker.scale.x = msg.scale * 0.11;
      marker.scale.y = msg.scale * 0.11;
      marker.scale.z = msg.scale * 1.1;
      marker.color.r = 0.2;
      marker.color.g = 0.9;
      marker.color.b = 0.2;
      marker.color.a = alpha;

      return marker;
    }

    /////
    /// @brief Creates a sphere-shaped marker.
    /// @param msg the interactive marker to associate this sphere with.
    /// @return the marker (which is a sphere)
    /////
    Marker makeMarkerSphere(InteractiveMarker &msg)
    {
      Marker marker;

      marker.type = Marker::SPHERE;
      // Scale is arbitrary.
      marker.scale.x = msg.scale * 0.75;
      marker.scale.y = msg.scale * 0.75;
      marker.scale.z = msg.scale * 0.75;
      marker.color.r = 0.8;
      marker.color.g = 0.8;
      marker.color.b = 1.0;
      marker.color.a = 0.1;

      return marker;
    }

    /////
    /// @brief Creates a clickable, box shaped marker.
    /// @param msg the interactive marker to associate this box with.
    /// @param alpha the transparency of the marker.
    /// @return the control (which is a clickable box)
    /////
    InteractiveMarkerControl& makeInteractiveBoxControl(InteractiveMarker &msg, float alpha = 1.0f)
    {
      InteractiveMarkerControl control;
      control.always_visible = true;
      control.markers.push_back(makeMarkerBox(msg, alpha));
      msg.controls.push_back(control);
      return msg.controls.back();
    }

    /////
    /// @brief Creates a clickable, cylinder shaped marker.
    /// @param msg the interactive marker to associate this cylinder with.
    /// @param alpha the transparency of the marker.
    /// @return the control (which is a clickable cylinder)
    /////
    InteractiveMarkerControl& makeInteractiveCylinderControl(InteractiveMarker &msg, float alpha = 1.0f)
    {
      InteractiveMarkerControl control;
      control.always_visible = true;
      control.markers.push_back(makeMarkerCylinder(msg, alpha));
      msg.controls.push_back(control);
      return msg.controls.back();
    }

    /////
    /// @brief Sends all collision pole changes and changes to the robot state to the planning environment.
    ////
    void refreshEnvironment()
    {
      GroupCollection& gc = group_map_[current_group_name_];
      sendPlanningScene();
      moveEndEffectorMarkers(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

      tf::Transform cur = toBulletTransform(last_ee_poses_[current_group_name_]);
      setNewEndEffectorPosition(gc, cur, collision_aware_);
    }

    /////
    /// @brief Main function that handles interactive marker feedback of all kinds.
    /// @param feedback the feedback message received by ROS.
    /// TODO: Replace this with several feedback functions.
    /////
    void processInteractiveFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
    {
      GroupCollection& gc = group_map_[current_group_name_];
      switch (feedback->event_type)
      {
        case InteractiveMarkerFeedback::BUTTON_CLICK:
          if(feedback->marker_name.rfind("_selectable") != string::npos)
          {
            if(feedback->marker_name == current_group_name_) {
              return;
            } 
            tf::Transform cur = toBulletTransform(feedback->pose);
            if(feedback->marker_name.rfind("pole_") != string::npos)
            {
              selectMarker(selectable_markers_[feedback->marker_name], cur);
            } else {
              deselectMarker(selectable_markers_[current_group_name_ + "_selectable"],
                             gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
              if(isGroupName(feedback->marker_name.substr(0, feedback->marker_name.rfind("_selectable"))))
              {
                unsigned int cmd = 0;
                for(map<string, GroupCollection>::iterator it = group_map_.begin(); it != group_map_.end(); it++)
                {
                  
                  if(feedback->marker_name.substr(0, feedback->marker_name.rfind("_selectable")) == it->first)
                  {
                    deleteKinematicStates();
                    selectPlanningGroup(cmd);
                    break;
                  } else {
                    
                  }
                  cmd++;
                }
              }
            }
          }
          break;

        case InteractiveMarkerFeedback::MENU_SELECT:
          MenuHandler::EntryHandle handle;
          if(feedback->marker_name.rfind("_selectable") != string::npos)
          {
            tf::Transform cur = toBulletTransform(feedback->pose);
            if(is_ik_control_active_
                && isGroupName(feedback->marker_name.substr(0, feedback->marker_name.rfind("_selectable"))))
            {
              handle = feedback->menu_entry_id;

              unsigned int cmd = 0;
              for(map<string, GroupCollection>::iterator it = group_map_.begin(); it != group_map_.end(); it++)
              {

                if(feedback->marker_name.substr(0, feedback->marker_name.rfind("_selectable")) == it->first)
                {
                  deleteKinematicStates();
                  selectPlanningGroup(cmd);
                  break;
                }
                cmd++;
              }

            }
            else if(feedback->marker_name.rfind("pole_") != string::npos)
            {
              handle = feedback->menu_entry_id;

              if(menu_entry_maps_["Collision Object Selection"][handle] == "Select")
              {
                selectMarker(selectable_markers_[feedback->marker_name], cur);
              }
              else if(menu_entry_maps_["Collision Object Selection"][handle] == "Delete")
              {
                this->removeCollisionPoleByName(
                                                feedback->marker_name.substr(0,
                                                                             feedback->marker_name.rfind("_selectable")));
                interactive_marker_server_->erase(feedback->marker_name);
                refreshEnvironment();
              }
            }
          }
          else if(feedback->marker_name == "top_level")
          {
            handle = feedback->menu_entry_id;
            if(handle == ik_control_handle_)
            {
              MenuHandler::CheckState currentState;
              menu_handler_map_["Top Level"].getCheckState(ik_control_handle_, currentState);

              if(currentState == MenuHandler::UNCHECKED)
              {
                is_ik_control_active_ = true;
                menu_handler_map_["Top Level"].setCheckState(ik_control_handle_, MenuHandler::CHECKED);
                menu_handler_map_["Top Level"].reApply(*interactive_marker_server_);
                selectMarker(selectable_markers_[current_group_name_ + "_selectable"],
                             gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
              }
              else
              {
                is_ik_control_active_ = false;
                menu_handler_map_["Top Level"].setCheckState(ik_control_handle_, MenuHandler::UNCHECKED);
                menu_handler_map_["Top Level"].reApply(*interactive_marker_server_);
                interactive_marker_server_->erase(current_group_name_);
                interactive_marker_server_->applyChanges();
              }
            }
            else if(handle == joint_control_handle_)
            {
              MenuHandler::CheckState currentState;
              menu_handler_map_["Top Level"].getCheckState(joint_control_handle_, currentState);
              if(currentState == MenuHandler::UNCHECKED)
              {
                is_joint_control_active_ = true;
                menu_handler_map_["Top Level"].setCheckState(joint_control_handle_, MenuHandler::CHECKED);
                createSelectableJointMarkers(gc);
                menu_handler_map_["Top Level"].reApply(*interactive_marker_server_);
              }
              else
              {
                is_joint_control_active_ = false;
                menu_handler_map_["Top Level"].setCheckState(joint_control_handle_, MenuHandler::UNCHECKED);
                deleteJointMarkers(gc);
                menu_handler_map_["Top Level"].reApply(*interactive_marker_server_);
              }
            }
            else if(handle == collision_aware_handle_)
            {
              MenuHandler::CheckState currentState;
              menu_handler_map_["Top Level"].getCheckState(collision_aware_handle_, currentState);
              if(currentState == MenuHandler::UNCHECKED)
              {
                collision_aware_ = true;
                menu_handler_map_["Top Level"].setCheckState(collision_aware_handle_, MenuHandler::CHECKED);
                createSelectableJointMarkers(gc);
                menu_handler_map_["Top Level"].reApply(*interactive_marker_server_);
              }
              else if(currentState == MenuHandler::CHECKED)
              {
                collision_aware_ = false;
                menu_handler_map_["Top Level"].setCheckState(collision_aware_handle_, MenuHandler::UNCHECKED);
                createSelectableJointMarkers(gc);
                menu_handler_map_["Top Level"].reApply(*interactive_marker_server_);
              }
            }
            if(menu_entry_maps_["Top Level"][handle] == "Create Pole")
            {
              Pose polePose;
              polePose.position.x = 2.0f;
              polePose.position.z = 1.0f;
              polePose.position.y = 0.0f;
              polePose.orientation.x = 0.0f;
              polePose.orientation.y = 0.0f;
              polePose.orientation.z = 0.0f;
              polePose.orientation.w = 1.0f;

              createCollisionPole(nextCollisionPole(), polePose);
              refreshEnvironment();
            }

            unsigned int cmd = 0;
            for(map<string, GroupCollection>::iterator it = group_map_.begin(); it != group_map_.end(); it++)
            {

              if(menu_entry_maps_["Top Level"][handle] == it->first)
              {
                selectPlanningGroup(cmd);
                break;
              }
              cmd++;
            }

          }
          else if(is_ik_control_active_ && isGroupName(feedback->marker_name))
          {
            handle = feedback->menu_entry_id;
            MenuHandler::CheckState checkState;

            if(handle == start_position_handle_ || handle == end_position_handle_)
            {
              menu_handler_map_["End Effector"].getCheckState(handle, checkState);

              if(checkState != MenuHandler::CHECKED)
              {
                menu_handler_map_["End Effector"].setCheckState(handle, MenuHandler::CHECKED);
                menu_handler_map_["End Effector"].reApply(*interactive_marker_server_);
              }

              if(handle == start_position_handle_)
              {
                menu_handler_map_["End Effector"].setCheckState(end_position_handle_, MenuHandler::UNCHECKED);
                ik_control_type_ = StartPosition;
                selectMarker(selectable_markers_[feedback->marker_name + "_selectable"],
                             gc.start_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
                createSelectableJointMarkers(gc);
                menu_handler_map_["End Effector"].reApply(*interactive_marker_server_);
              }
              else if(handle == end_position_handle_)
              {
                menu_handler_map_["End Effector"].setCheckState(start_position_handle_, MenuHandler::UNCHECKED);
                ik_control_type_ = EndPosition;
                selectMarker(selectable_markers_[feedback->marker_name + "_selectable"],
                             gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
                createSelectableJointMarkers(gc);
                menu_handler_map_["End Effector"].reApply(*interactive_marker_server_);
              }

              updateJointStates(gc);

            } 
            else if(handle == constrain_rp_handle_) {
              menu_handler_map_["End Effector"].getCheckState(handle, checkState);
              if(checkState == MenuHandler::UNCHECKED)
              {
                menu_handler_map_["End Effector"].setCheckState(handle, MenuHandler::CHECKED);
                constrain_rp_ = true;
              } else {
                menu_handler_map_["End Effector"].setCheckState(handle, MenuHandler::UNCHECKED);
                constrain_rp_ = false;
              }
              moveEndEffectorMarkers(0.0,0.0,0.0,0.0,0.0,0.0,true);
              menu_handler_map_["End Effector"].reApply(*interactive_marker_server_);
            }
            else if(menu_entry_maps_["End Effector"][handle] == "Plan")
            {
              planToEndEffectorState(gc);
            }
            else if(menu_entry_maps_["End Effector"][handle] == "Filter Trajectory")
            {
              filterPlannerTrajectory(gc);
            }
            else if(menu_entry_maps_["End Effector"][handle] == "Randomly Perturb")
            {
              randomlyPerturb(gc);
            }
            else if(menu_entry_maps_["End Effector"][handle] == "Go To Last Good State")
            {
              resetToLastGoodState(gc);
            }
            else if(menu_entry_maps_["End Effector"][handle] == "Deselect")
            {
              tf::Transform cur = toBulletTransform(feedback->pose);
              deselectMarker(selectable_markers_[feedback->marker_name + "_selectable"], cur);
            }
          }
          else if(feedback->marker_name.rfind("pole_") != string::npos)
          {
            handle = feedback->menu_entry_id;

            stringstream controlName;
            controlName << feedback->marker_name;

            if(menu_entry_maps_["Collision Object"][handle] == "Delete")
            {
              this->removeCollisionPoleByName(feedback->marker_name);
              interactive_marker_server_->erase(feedback->marker_name);
              refreshEnvironment();
            }
            else if(menu_entry_maps_["Collision Object"][handle] == "Deselect")
            {
              tf::Transform cur = toBulletTransform(feedback->pose);
              deselectMarker(selectable_markers_[feedback->marker_name + "_selectable"], cur);
            }
          }

          break;

        case InteractiveMarkerFeedback::MOUSE_UP:
          prev_joint_control_value_map_.clear();
          if(feedback->marker_name.rfind("pole_") != string::npos && feedback->marker_name.rfind("_selectable")
              == string::npos)
          {
            collision_poles_[feedback->marker_name].poses[0] = feedback->pose;
            refreshEnvironment();
          }
          else if(feedback->marker_name.rfind("_joint_control") != string::npos)
            {
              joint_clicked_map_[feedback->marker_name] = false;
            }
          break;

        case InteractiveMarkerFeedback::MOUSE_DOWN:
          if(feedback->marker_name.rfind("_joint_control") != string::npos)
          {
            if(!joint_clicked_map_[feedback->marker_name])
            {
              joint_clicked_map_[feedback->marker_name] = true;
              joint_prev_transform_map_[feedback->marker_name] = toBulletTransform(feedback->pose);
            }
          }
          break;
        case InteractiveMarkerFeedback::POSE_UPDATE:
          if(is_ik_control_active_ && isGroupName(feedback->marker_name))
          {
            tf::Transform cur = toBulletTransform(feedback->pose);
            setNewEndEffectorPosition(gc, cur, collision_aware_);
            last_ee_poses_[current_group_name_] = feedback->pose;
          }
          else if(is_joint_control_active_ && feedback->marker_name.rfind("_joint_control") != string::npos)
          {
            tf::Transform cur = toBulletTransform(feedback->pose);
            string jointName = feedback->marker_name.substr(0, feedback->marker_name.rfind("_joint_control"));
            setJointState(gc, jointName, cur);
          }
          break;
      }
      interactive_marker_server_->applyChanges();
    }

    /////
    /// @brief Returns true if the string of the given name corresponds to a kinematic chain in the robot.
    /// @param name a string corresponding to a kinematic chain.
    /// @return true if the name is a kinematic chain, or false otherwise.
    /////
    bool isGroupName(const string& name)
    {
      return group_map_.find(name) != group_map_.end();
    }

    void makeTopLevelMenu()
    {
      InteractiveMarker int_marker;
      int_marker.pose.position.z = 2.25;
      int_marker.name = "top_level";
      int_marker.description = "Planning Visualizer";
      int_marker.header.frame_id = "/" + cm_->getWorldFrameId();


      InteractiveMarkerControl control;
      control.interaction_mode = InteractiveMarkerControl::MENU;
      control.always_visible = true;

      Marker labelMarker;
      labelMarker.type = Marker::TEXT_VIEW_FACING;
      labelMarker.text = "Command...";
      labelMarker.color.r = 1.0;
      labelMarker.color.g = 1.0;
      labelMarker.color.b = 1.0;
      labelMarker.color.a = 1.0;
      labelMarker.scale.x = 0.5;
      labelMarker.scale.y = 0.2;
      labelMarker.scale.z = 0.1;
      control.markers.push_back(labelMarker);

      int_marker.controls.push_back(control);

      interactive_marker_server_->insert(int_marker);
      interactive_marker_server_->setCallback(int_marker.name, process_function_ptr_);
      menu_handler_map_["Top Level"].apply(*interactive_marker_server_, int_marker.name);
    }

    ////
    /// @brief Wrapper for makeSelectableMarker which assumes we are creating a collision pole.
    ////
    void makePoleContextMenu(tf::Transform transform, string name, string description, float scale = 1.0f)
    {

      makeSelectableMarker(PlanningComponentsVisualizer::CollisionObject, transform, name, description, scale);
    }

    /////
    /// @brief Creates a marker that is initially a clickable menu. Upon selection it turns into a 6DOF control.
    /// @param type the type (collision object, joint, IK control, etc.) of the selectable marker.
    /// @param transform location and orientation of the marker.
    /// @param name internal, unique representation of the marker (this is for the 6DOF control)
    /// @param description displayed above the menu marker and the 6DOF marker.
    /// @param scale uniformly sizes the marker and its controls
    /// @param publish if true, the marker server will publish the marker. Otherwise, it will not.
    /////
    void makeSelectableMarker(InteractiveMarkerType type, tf::Transform transform, string name, string description,
                              float scale = 1.0f, bool publish = true)
    {
      SelectableMarker selectable_marker;
      selectable_marker.type_ = type;
      selectable_marker.name_ = name + "_selectable";
      selectable_marker.controlName_ = name;
      selectable_marker.controlDescription_ = description;

      InteractiveMarker marker;
      marker.header.frame_id = "/" + cm_->getWorldFrameId();
      ;
      marker.header.stamp = ros::Time::now();
      marker.pose.position.x = transform.getOrigin().x();
      marker.pose.position.y = transform.getOrigin().y();
      marker.pose.position.z = transform.getOrigin().z();
      marker.pose.orientation.w = transform.getRotation().w();
      marker.pose.orientation.x = transform.getRotation().x();
      marker.pose.orientation.y = transform.getRotation().y();
      marker.pose.orientation.z = transform.getRotation().z();
      marker.scale = scale;
      marker.name = name + "_selectable";
      marker.description = description;
      InteractiveMarkerControl control;
      control.interaction_mode = InteractiveMarkerControl::BUTTON;
      control.always_visible = true;

      switch (type)
      {
        case PlanningComponentsVisualizer::EndEffectorControl:
          control.markers.push_back(makeMarkerBox(marker, 0.5f));
          marker.controls.push_back(control);
          interactive_marker_server_->insert(marker);
          interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
          menu_handler_map_["End Effector Selection"].apply(*interactive_marker_server_, marker.name);
          break;
        case PlanningComponentsVisualizer::CollisionObject:
          control.markers.push_back(makeMarkerCylinder(marker, 1.0f));
          marker.controls.push_back(control);
          interactive_marker_server_->insert(marker);
          interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
          menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, marker.name);
          break;
        case PlanningComponentsVisualizer::JointControl:
          control.markers.push_back(makeMarkerBox(marker, 0.5f));
          marker.controls.push_back(control);
          interactive_marker_server_->insert(marker);
          interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
          menu_handler_map_["Joint Selection"].apply(*interactive_marker_server_, marker.name);
          break;
      }

      selectable_markers_[marker.name] = selectable_marker;

      if(publish)
      {
        interactive_marker_server_->applyChanges();
      }
    }

    ////
    /// @brief Returns true if a selectable marker of the given name exists in the marker map.
    /// @param name the unique identifier of the marker.
    /// @returns true if the marker exists, or false otherwise.
    ////
    bool selectableMarkerExists(string name)
    {
      return selectable_markers_.find(name) != selectable_markers_.end();
    }

    /////
    /// @brief Removes the menu marker given and replaces it with a 6DOF marker
    /// @param marker a reference to the selectablemarker struct.
    /// @param transform location to select the marker.
    /////
    void selectMarker(SelectableMarker& marker, tf::Transform transform)
    {
      InteractiveMarker dummy;
      if(interactive_marker_server_->get(marker.controlName_, dummy))
      {
        dummy.header.stamp = ros::Time::now();
        interactive_marker_server_->setPose(marker.controlName_, toGeometryPose(transform), dummy.header);
      }
      else
      {
        if(!interactive_marker_server_->erase(marker.name_))
        {
          return;
        }

        switch (marker.type_)
        {
          case PlanningComponentsVisualizer::EndEffectorControl:
            makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
            break;
          case PlanningComponentsVisualizer::CollisionObject:
            makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 2.0f, true);
            break;
          case PlanningComponentsVisualizer::JointControl:
            makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
            break;
        }
      }
    }

    /////
    /// @brief Removes the 6DOF control of the given marker and replaces it with a menu.
    /// @param marker a reference to the selectablemarker struct.
    /// @param transform location of the marker when it is de-selected.
    /////
    void deselectMarker(SelectableMarker& marker, tf::Transform transform)
    {
      if(!interactive_marker_server_->erase(marker.controlName_))
      {
        return;
      }

      float scale = 1.0f;

      switch (marker.type_)
      {
        case PlanningComponentsVisualizer::EndEffectorControl:
          scale = 0.5f;
          break;
        case PlanningComponentsVisualizer::CollisionObject:
          scale = 2.0f;
          break;
        case PlanningComponentsVisualizer::JointControl:
          scale = 0.225f;
          break;
      }

      makeSelectableMarker(marker.type_, transform, marker.controlName_, marker.controlDescription_, scale);
    }

    void makeInteractive1DOFTranslationMarker(tf::Transform transform, tf::Vector3 axis, string name, string description,
                                              float scale = 1.0f, float value = 0.0f)
    {
      InteractiveMarker marker;
      marker.header.frame_id = cm_->getWorldFrameId();
      marker.pose.position.x = transform.getOrigin().x();
      marker.pose.position.y = transform.getOrigin().y();
      marker.pose.position.z = transform.getOrigin().z();
      marker.pose.orientation.w = transform.getRotation().w();
      marker.pose.orientation.x = transform.getRotation().x();
      marker.pose.orientation.y = transform.getRotation().y();
      marker.pose.orientation.z = transform.getRotation().z();
      marker.scale = scale;
      marker.name = name;
      marker.description = description;
      InteractiveMarker dummy;
      InteractiveMarkerControl control;
      if(interactive_marker_server_->get(marker.name, dummy))
      {
        interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
      }
      else
      {
        control.orientation.x = axis.x();
        control.orientation.y = axis.z();
        control.orientation.z = axis.y();
        control.orientation.w = 1;
        control.independent_marker_orientation = false;
        control.always_visible = false;
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);
        interactive_marker_server_->insert(marker);
        interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
      }

    }

    void makeInteractive1DOFRotationMarker(tf::Transform transform, tf::Vector3 axis, string name, string description,
                                           float scale = 1.0f, float angle = 0.0f)
    {
      InteractiveMarker marker;
      marker.header.frame_id = cm_->getWorldFrameId();
      marker.pose.position.x = transform.getOrigin().x();
      marker.pose.position.y = transform.getOrigin().y();
      marker.pose.position.z = transform.getOrigin().z();
      marker.pose.orientation.w = transform.getRotation().w();
      marker.pose.orientation.x = transform.getRotation().x();
      marker.pose.orientation.y = transform.getRotation().y();
      marker.pose.orientation.z = transform.getRotation().z();
      marker.scale = scale;
      marker.name = name;
      marker.description = description;

      InteractiveMarker dummy;
      if(interactive_marker_server_->get(marker.name, dummy))
      {
        interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
      }
      else
      {
        InteractiveMarkerControl control;
        control.orientation.x = axis.x();
        control.orientation.y = axis.z();
        control.orientation.z = axis.y();
        control.orientation.w = 1;
        control.independent_marker_orientation = false;
        control.always_visible = false;
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(control);
        interactive_marker_server_->insert(marker);
        interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
      }
    }

    /////
    /// @brief Creates an interactive 6DOF marker control.
    /// @param fixed if true, the axes remain fixed to the frame of the marker. Otherwise they rotate with input.
    /// @param transform the location and rotation of the 6DOF marker.
    /// @param name used internally to represent the marker. Must be unique.
    /// @param description drawn above the marker as a label.
    /// @param scale uniformly sets the size in meters of the marker.
    /// @param pole if true, the marker is a large green cylinder. Otherwise it is a small white cube.
    /////
    void makeInteractive6DOFMarker(bool fixed, tf::Transform transform, string name, string description,
                                   float scale = 1.0f, bool pole = false)
    {
      InteractiveMarker marker;
      marker.header.frame_id = "/" + cm_->getWorldFrameId();
      marker.pose.position.x = transform.getOrigin().x();
      marker.pose.position.y = transform.getOrigin().y();
      marker.pose.position.z = transform.getOrigin().z();
      marker.pose.orientation.w = transform.getRotation().w();
      marker.pose.orientation.x = transform.getRotation().x();
      marker.pose.orientation.y = transform.getRotation().y();
      marker.pose.orientation.z = transform.getRotation().z();
      marker.scale = scale;
      marker.name = name;
      marker.description = description;

      if(!pole)
      {
        makeInteractiveBoxControl(marker, 0.5f);
      }
      else
      {
        makeInteractiveCylinderControl(marker, 1.0f);
      }

      InteractiveMarkerControl control;

      if(fixed)
      {
        control.orientation_mode = InteractiveMarkerControl::FIXED;
      }

      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.always_visible = false;
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      marker.controls.push_back(control);
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      marker.controls.push_back(control);
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      marker.controls.push_back(control);
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      marker.controls.push_back(control);

      interactive_marker_server_->insert(marker);

      control.interaction_mode = InteractiveMarkerControl::MENU;
      //control.markers.push_back(makeMarkerSphere(marker));
      marker.controls.push_back(control);

      if(!pole)
      {
       menu_handler_map_["End Effector"].apply(*interactive_marker_server_, marker.name);
      }
      else
      {
       menu_handler_map_["Collision Object"].apply(*interactive_marker_server_, marker.name);
      }

      interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
    }

    bool doesGroupHaveGoodIKSolution(const string& group) const
    {
      if(group_map_.find(group) == group_map_.end())
      {
        return false;
      }
      return group_map_.find(group)->second.good_ik_solution_;
    }

    bool doesGroupHaveGoodTrajectory(const string& group, const string& source) const
    {
      if(group_map_.find(group) == group_map_.end())
      {
        return false;
      }
      const GroupCollection& gc = group_map_.find(group)->second;
      if(gc.state_trajectory_display_map_.find(source) == gc.state_trajectory_display_map_.end())
      {
        return false;
      }
      return gc.state_trajectory_display_map_.find(source)->second.has_joint_trajectory_;
    }

    /////
    /// Returns the number of kinematic chains in the robot.
    /// @return size_t with number of chains of the robot.
    /////
    size_t getNumPlanningGroups()
    {
      return group_map_.size();
    }

    GroupCollection* getPlanningGroup(unsigned int i)
    {
      unsigned int cmd = 0;
      for(map<string, GroupCollection>::iterator it = group_map_.begin(); it != group_map_.end(); it++)
      {
        if(cmd == i)
        {
          return &(it->second);
        }
        cmd++;
      }

      return NULL;
    }
  protected:

    Pose toGeometryPose(tf::Transform transform)
    {
      Pose toReturn;
      toReturn.position.x = transform.getOrigin().x();
      toReturn.position.y = transform.getOrigin().y();
      toReturn.position.z = transform.getOrigin().z();
      toReturn.orientation.x = transform.getRotation().x();
      toReturn.orientation.y = transform.getRotation().y();
      toReturn.orientation.z = transform.getRotation().z();
      toReturn.orientation.w = transform.getRotation().w();
      return toReturn;
    }

    tf::Transform toBulletTransform(geometry_msgs::Pose pose)
    {
      tf::Quaternion quat = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      tf::Vector3 vec = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
      return tf::Transform(quat, vec);
    }

    void deleteKinematicStates()
    {
      for(map<string, GroupCollection>::iterator it = group_map_.begin(); it != group_map_.end(); it++)
      {
        it->second.reset();
      }
    }

    IKControlType ik_control_type_;


  boost::recursive_mutex joint_state_lock_;
  sensor_msgs::JointState last_joint_state_msg_;

    boost::recursive_mutex lock_;
    boost::shared_ptr<InteractiveMarkerServer> interactive_marker_server_;

    CollisionModels* cm_;

    /// Used to generate new IDs for collision poles. Not the actual number of poles.
    int num_collision_poles_;

    KinematicState* robot_state_;

    map<string, GroupCollection> group_map_;

    /// Map of collision pole names to messages sent to ROS.
    map<string, arm_navigation_msgs::CollisionObject> collision_poles_;

    /// Maps end effector link names to their previously recorded poses.
    map<string, Pose> last_ee_poses_;

    /// Maps selectable marker names to a struct containing their information.
    map<string, SelectableMarker> selectable_markers_;

    /// Boost function pointer to the main interactive feedback function.
    MenuHandler::FeedbackCallback process_function_ptr_;

    MenuHandler::EntryHandle start_position_handle_;
    MenuHandler::EntryHandle end_position_handle_;
    MenuHandler::EntryHandle ik_control_handle_;
    MenuHandler::EntryHandle joint_control_handle_;
    MenuHandler::EntryHandle collision_aware_handle_;
    MenuHandler::EntryHandle constrain_rp_handle_;
    bool constrain_rp_;
    bool collision_aware_;
    /// Maps strings to menu handlers. This is used for convenience and extensibility.
    MenuHandlerMap menu_handler_map_;

    /// Maps MenuHandles to their names. Used to determine which menu entry is selected.
    MenuMap menu_entry_maps_;

    MotionPlanRequest last_motion_plan_request_;

    ros::NodeHandle nh_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher vis_marker_array_publisher_;
    ros::Publisher vis_marker_publisher_;
    ros::ServiceClient set_planning_scene_diff_client_;
    ros::ServiceClient planner_service_client_;
    ros::ServiceClient trajectory_filter_service_client_;

    string current_group_name_;

    tf::TransformBroadcaster transform_broadcaster_;

    bool is_ik_control_active_;
    bool is_joint_control_active_;

    map<string, bool> joint_clicked_map_;
    map<string, tf::Transform> joint_prev_transform_map_;
  map<string, double> prev_joint_control_value_map_;
};

PlanningComponentsVisualizer* pcv;

bool inited = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while(ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }
}

void update_function()
{
  unsigned int counter = 0;
  while(ros::ok())
  {
    if(inited)
    {
      //pcv->sendTransforms();
      if(counter % CONTROL_SPEED == 0)
      {
        counter = 1;
        pcv->sendMarkers();
      }
      else
      {
        pcv->publishJointStates();
        counter++;
      }
    }

    usleep(5000);
  }
}

void quit(int sig)
{
  if(pcv != NULL)
  {
    delete pcv;
  }
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_components_visualizer", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));
  boost::thread update_thread(boost::bind(&update_function));
  pcv = new PlanningComponentsVisualizer();

  inited = true;

  for(size_t i = 0; i < pcv->getNumPlanningGroups(); i++)
  {
    pcv->selectPlanningGroup(i);
    pcv->solveIKForEndEffectorPose((*pcv->getPlanningGroup(i)));
    pcv->updateJointStates((*pcv->getPlanningGroup(i)));
  }

  ros::waitForShutdown();

  return 0;
}
