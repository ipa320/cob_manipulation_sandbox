import roslib
roslib.load_manifest('cob_arm_navigation_python')
import rospy

from arm_navigation_msgs.srv import *
from arm_navigation_msgs.msg import *
from kinematics_msgs.srv import *
from control_msgs.msg import *
from cob_kinematics.srv import *
from MotionPlan import *
import actionlib
#from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from copy import deepcopy
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_python.trajectory_tools import last_state_on_joint_trajectory
from tf.transformations import *

arm_nav_error_dict = {}
for name,val in arm_navigation_msgs.msg.ArmNavigationErrorCodes.__dict__.items():
    if not name[:1] == '_' and name != 'val':
        arm_nav_error_dict[val] = name

def parse_cartesian_param(param, now = None):
    if now is None:
        now = rospy.Time.now()
    ps = PoseStamped()
    ps.pose.orientation.w = 1.0
    ps.header.stamp = now
    if type(param) is not PoseStamped and param is not None:
        ps.header.frame_id = param[0]
        if len(param) > 1:
            ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = param[1]
        if len(param) > 2:
            ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quaternion_from_euler(*param[2])
    else:
        ps = param
    return ps
    
def parse_cartesian_parameters(arm_name, parameters):
    now = rospy.Time.now()
    
    # parse pose_target
    param = parameters
    second_param = None
    if type(parameters) is list and len(parameters) > 0:
        if type(parameters[0]) is not str:
            param = parameters[0]
            if len(parameters) > 1:
                second_param = parameters[1]

    pose_target = parse_cartesian_param(param, now)

    # parse pose_origin
    param = second_param
    ps = PoseStamped()
    ps.pose.orientation.w = 1.0
    ps.header.stamp = pose_target.header.stamp
    ps.header.frame_id = rospy.get_param("/cob_arm_kinematics/"+arm_name+"/tip_name")
    if type(param) is not PoseStamped:
         if param is not None and len(param) >=1:
            ps.header.frame_id = param[0]
            if len(param) > 1:
                ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = param[1]
            if len(param) > 2:
                ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quaternion_from_euler(*param[2])
    else:
        ps = param
    return pose_target,ps
    
def get_joint_goal(arm_name, target, robot_state):
    cart_goal = False
    try:
        ps_target, ps_origin = parse_cartesian_parameters(arm_name, target)
        cart_goal = "link" in ps_target.header.frame_id or ps_target.header.frame_id == "odom_combined" or ps_target.header.frame_id == "map" #todo: use tf?
    except (KeyError, ValueError, TypeError): 
        cart_goal = False

    if cart_goal:
        iks = rospy.ServiceProxy("/cob_ik_wrapper/arm/get_ik_extended", GetPositionIKExtended)
        req = GetPositionIKExtendedRequest()
        req.ik_pose = ps_origin.pose
        req.constraint_aware = True
        req.timeout = rospy.Duration(5.0)
        req.ik_request.ik_link_name = ps_origin.header.frame_id
        req.ik_request.pose_stamped = ps_target
        req.ik_request.ik_seed_state = robot_state
        req.ik_request.robot_state = robot_state
        res = iks(req)
        return res.solution.joint_state, res.error_code
    else:
        return read_target_state_from_param(arm_name, target), None

class MoveArm(MotionExecutable):
    def __init__(self, name, target, constraint_aware = True):
        self.name = name
        self.target = target
        self.goal = None
        self.constraint_aware = constraint_aware
        self.planner = rospy.ServiceProxy("/cob_arm_navigation/cache_motion_plan", GetMotionPlan)
        self.planner_service_name = "/cob_arm_navigation/get_cached_motion_plan"
        try:
            self.planner.wait_for_service(0.1)
        except:
            self.planner = rospy.ServiceProxy("/ompl_planning/plan_kinematic_path", GetMotionPlan)
            self.planner_service_name = "/ompl_planning/plan_kinematic_path"
        rospy.loginfo("Using " + self.planner_service_name)
    def plan(self, update_ps = True):
        psi = get_planning_scene_interface()

        joint_goal, err = get_joint_goal(self.name, self.target, psi.get_robot_state())
        #print joint_goal
        if err is not None and err.val != err.SUCCESS:
            self.goal = None
            return ErrorCode("IK error: " + arm_nav_error_dict[err.val])
            
        req = GetMotionPlanRequest()
        req.motion_plan_request.group_name = self.name
        req.motion_plan_request.num_planning_attempts = 1
        req.motion_plan_request.allowed_planning_time = rospy.Duration(50.0)

        req.motion_plan_request.planner_id= ""
        #req.motion_plan_request.start_state = planning_scene.get_current_scene().robot_state

        for n,p in zip(joint_goal.name, joint_goal.position):
            new_constraint = JointConstraint()
            new_constraint.joint_name = n
            new_constraint.position = p
            new_constraint.tolerance_below = 0.05
            new_constraint.tolerance_above = 0.05
            req.motion_plan_request.goal_constraints.joint_constraints.append(new_constraint)
            
        if self.constraint_aware:
            res = self.planner(req)
        if not self.constraint_aware or res.error_code.val == res.error_code.SUCCESS:
            self.goal = MoveArmGoal()
            self.goal.planner_service_name = self.planner_service_name
            self.goal.motion_plan_request = req.motion_plan_request
            if self.constraint_aware:
                self.goal.operations = deepcopy(psi.current_diff.operations)
            else:
                op = CollisionOperation()
                op.object1 = op.COLLISION_SET_OBJECTS
                op.object2 = op.COLLISION_SET_ATTACHED_OBJECTS
                op.operation = op.DISABLE
                self.goal.operations.collision_operations.append(op)
            if update_ps:
                if self.constraint_aware:
                    set_planning_scene_joint_state(last_state_on_joint_trajectory(res.trajectory.joint_trajectory))
                else:
                    set_planning_scene_joint_state(joint_goal)
            #print "points:", res.trajectory.joint_trajectory.points
            return ErrorCode()
        else:
            self.goal = None
            return ErrorCode(arm_nav_error_dict[res.error_code.val])

    def execute(self):
        if self.goal is None:
            error_code = self.plan(update_ps=False)
            if not error_code.success:
                raise error_code
        client = actionlib.SimpleActionClient("/move_"+self.name, MoveArmAction)
        return MotionHandle(client, self.goal)
    

"""class MoveArmInterpolated(MotionExecutable):
    def __init__(self, name, target):
        self.name = name
        self.target = target
        self.motion_plan = None
        self.planner = rospy.ServiceProxy("/arm_interpolated_ik_motion_plan", GetMotionPlan)
    def plan(self, planning_scene=None):
        if planning_scene == None:
            planning_scene = PlanningScene.PlanningScene()
        target_pose = parse_cartesian_param(self.target)
        
        req = GetMotionPlanRequest()
        req.motion_plan_request.group_name = self.name
        req.motion_plan_request.num_planning_attempts = 1
        req.motion_plan_request.allowed_planning_time = rospy.Duration(50.0)

        req.motion_plan_request.planner_id= ""

        position_constraint = PositionConstraint()
        position_constraint.position = target_pose.pose.position
        position_constraint.header.frame_id = target_pose.header.frame_id
        req.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        req.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)
        
        req.motion_plan_request.start_state = planning_scene.get_current_scene().robot_state
        res = self.planner(req)
        print res
        if res.error_code.val == res.error_code.SUCCESS:
            self.motion_plan = res
            names = res.trajectory.joint_trajectory.joint_names
            positions = res.trajectory.joint_trajectory.points[-1].positions
            planning_scene.set_joint_state(names, positions)
            #print "points:", res.trajectory.joint_trajectory.points
            return ErrorCode()
        else:
            self.motion_plan = None
            return ErrorCode(arm_nav_error_dict[res.error_code.val])
    def execute(self):
        if self.motion_plan is None:
            error_code = self.plan()
            if not error_code.success:
                raise error_code
    
        client = actionlib.SimpleActionClient(self.name + '_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "/"+self.name + '_controller/follow_joint_trajectory'
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.motion_plan.trajectory.joint_trajectory
        for i in range(len(goal.trajectory.points)):
            goal.trajectory.points[i].time_from_start=rospy.Duration(1)
        return MotionHandle(client, goal)
"""
