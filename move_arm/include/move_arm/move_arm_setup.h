/*********************************************************************
 *
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 * \author Sachin Chitta, Ioan Sucan
 *********************************************************************/

#ifndef MOVE_ARM_MOVE_ARM_SETUP_
#define MOVE_ARM_MOVE_ARM_SETUP_

#include <ros/ros.h>

#include <arm_control_msgs/TrajectoryStart.h>
#include <arm_control_msgs/TrajectoryQuery.h>
#include <arm_control_msgs/TrajectoryCancel.h>

#include <planning_environment/monitors/planning_monitor.h>

namespace move_arm
{

    /// the string used internally to access control starting service; this should be remaped in the launch file
    static const std::string CONTROL_START_NAME      = "controller_start";

    /// the string used internally to access control querying service; this should be remaped in the launch file
    static const std::string CONTROL_QUERY_NAME      = "controller_query";

    /// the string used internally to access control canceling service; this should be remaped in the launch file
    static const std::string CONTROL_CANCEL_NAME     = "controller_cancel";

    /// the string used internally to access valid state searching service; this should be remaped in the launch file
    static const std::string SEARCH_VALID_STATE_NAME = "get_valid_state";

    /// the string used internally to access inverse kinematics service; this should be remaped in the launch file
    static const std::string ARM_IK_NAME             = "arm_ik";


    /** \brief Configuration of actions that need to actuate parts of the robot */
    class MoveArmSetup
    {
	friend class MoveArm;
        friend class MoveArmMonitor;
	friend class TeleopArm;

    public:

	MoveArmSetup(void): nodeHandle_("~")
	{
	    collision_models_ = NULL;
	    planning_monitor_ = NULL;
	}

	virtual ~MoveArmSetup(void)
	{
	    if (planning_monitor_)
		delete planning_monitor_;
	    if (collision_models_)
		delete collision_models_;
	}


	bool configure(void);

    protected:

	bool getControlJointNames(std::vector<std::string> &joint_names);

	ros::NodeHandle                        nodeHandle_, root_handle_;
	tf::TransformListener                  tf_;
	planning_environment::CollisionModels *collision_models_;
	planning_environment::PlanningMonitor *planning_monitor_;

	std::string                            group_;
	std::vector<std::string>               groupJointNames_;

      private:
        bool use_collision_map_;

    };

}

#endif
