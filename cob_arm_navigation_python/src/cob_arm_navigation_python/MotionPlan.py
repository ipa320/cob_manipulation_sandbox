import roslib
roslib.load_manifest('cob_arm_navigation_python')

import rospy
import threading

from actionlib_msgs.msg import *
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_python.trajectory_tools import set_joint_state_in_robot_state

from geometry_msgs.msg import *
from sensor_msgs.msg import *


from arm_navigation_msgs.msg import *

arm_nav_error_dict = {}
for name,val in arm_navigation_msgs.msg.ArmNavigationErrorCodes.__dict__.items():
    if not name[:1] == '_' and name != 'val':
        arm_nav_error_dict[val] = name


def read_target_state_from_param(component_name, parameter_name):
    # get joint_names from parameter server
    sss_global_prefix = "/script_server"
    param_string = sss_global_prefix + "/" + component_name + "/joint_names"
    if not rospy.has_param(param_string):
        return None
    joint_names = rospy.get_param(param_string)
    
    # check joint_names parameter
    if not type(joint_names) is list: # check list
        return None
        
    for i in joint_names:
        #print i,"type1 = ", type(i)
        if not type(i) is str: # check string
            rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",component_name)
            print "joint_names are:",param
            return None
        else:
            rospy.logdebug("accepted joint_names for component %s",component_name)
    
    # get joint values from parameter server
    if type(parameter_name) is str:
        if not rospy.has_param(sss_global_prefix + "/" + component_name + "/" + parameter_name):
            rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",sss_global_prefix + "/" + component_name + "/" + parameter_name)
            return None
        param = rospy.get_param(sss_global_prefix + "/" + component_name + "/" + parameter_name)
    else:
        param = parameter_name

    # check trajectory parameters
    if not type(param) is list: # check outer list
        rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
        print "parameter is:",param
        return None

    traj = []

    for point in param:
        #print point,"type1 = ", type(point)
        if type(point) is str:
            if not rospy.has_param(sss_global_prefix + "/" + component_name + "/" + point):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",sss_global_prefix + "/" + component_name + "/" + point)
                return None
            point = rospy.get_param(sss_global_prefix + "/" + component_name + "/" + point)
            point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
            #print point
        elif type(point) is list:
            rospy.logdebug("point is a list")
        else:
            rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
            print "parameter is:",param
            return None

        # here: point should be list of floats/ints
        #print point
        if not len(point) == len(joint_names): # check dimension
            rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(point))
            print "parameter is:",param
            return None

        for value in point:
            #print value,"type2 = ", type(value)
            if not ((type(value) is float) or (type(value) is int)): # check type
                #print type(value)
                rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
                print "parameter is:",param
                return None
        
            rospy.logdebug("accepted value %f for %s",value,component_name)
        traj.append(point)
    return JointState(name=joint_names, position=traj[-1])
    
def set_planning_scene_joint_state(js):
    psi = get_planning_scene_interface()
    rs = psi.get_robot_state()
    set_joint_state_in_robot_state(js,rs)
    psi.set_robot_state(rs)
    psi.send_diff()
    
class ErrorCode(Exception):
    def __init__(self, msg=None):
        self.error_code = msg
        self.success = msg is None
    def __str__(self):
        if self.success:
            return "SUCCESS"
        else:
            return "ERROR: "+str(self.error_code)

class MotionHandleDummy:
    def __init__(self, msg = None):
        self.err = ErrorCode(msg)
    def retry(self):
        pass
    def cancel(self):
        pass
    def is_done(self):
        return 1
    def wait(self, duration = None, hz=100):
        return self.err
        
class MotionHandle:
    def __init__(self, client, goal):
        self.client = client
        self.goal = goal
        self.done = 0
        self.state = None
        self.result = None	
        self.retry()
    def done_cb(self, state, result):
        self.state = state
        self.result = result
        if  state == GoalStatus.SUCCEEDED:
            self.done = 1
        elif state == GoalStatus.ABORTED:
            self.done = -1
    def retry(self):
        self.client.wait_for_server(rospy.Duration(5.0))
        self.client.cancel_all_goals()
        self.client.send_goal(self.goal, done_cb=self.done_cb)
        rospy.loginfo("sent goal")
    def cancel(self):
        self.client.cancel_all_goals()
    def is_done(self):
        return self.done
    def wait(self, duration = None, hz=100):
        r = rospy.Rate(hz)
        if duration is not None:
            d = rospy.Duration(duration)
        start = rospy.Time.now() 
        while self.is_done() == 0 and (duration is None or d > rospy.Time.now() - start):
            #print self.client.get_state()
            r.sleep()
        print "MotionHanldeState: ", self.state
        print "MotionHandleResult: ", self.result        
        if self.is_done() == 1:
            return ErrorCode()
        self.cancel()
        return ErrorCode("Motion failure %d: %s" % (self.client.get_state(), self.client.get_goal_status_text()))        
class MotionHandleSSS(MotionHandle):
    def __init__(self, sss, command):
        self.sss = sss
        self.done = 0 
        self.command = command
        self.handle = self.sss.move(*command, blocking=False)
        self.client = self.handle.client
    def retry(self):
        self.done = 0 
        self.handle.cancel()
        self.handle = self.sss.move(*command, blocking=False)
        self.client = self.handle.client
    def is_done(self):
        self.done_cb(self.handle.get_state(),None)
        return self.done
    def cancel(self):
        self.handle.cancel()    
        
class MotionExecutable:
    def __radd__(self):
        return [self]
    def info(self):
        pass
class CallFunction(MotionExecutable):
    def __init__(self, callback, *args, **kwargs):
        self.callback = callback
        self.args = args
        self.kwargs = kwargs
        self.type = "CallFunction"
    def plan(self, psi=None):
        return ErrorCode()
    def execute(self):
        self.callback(*self.args, **self.kwargs)
        return MotionHandleDummy()
        
class MotionPlan:
    def __init__(self):
        self.executables = []
    def __iadd__(self, ex):
        self.executables.append(ex)
        return self
    def plan(self, retries = 1):
        for ex in self.executables:
            print "\nStart planning for executable " + ex.type
            ex.info()
            for i in range(retries+1):
                error_code = ex.plan()
                print "Try Nr. ", i, error_code
                if error_code.success:
                    break
            if not error_code.success:
                return error_code
        print "Done planning"
        return ErrorCode()

    def execute(self):
        get_planning_scene_interface().reset()
        for ex in self.executables:
            print "\nStart executing executable " + ex.type
            yield ex.execute()
