import roslib
roslib.load_manifest('cob_arm_navigation_python')
import rospy

from arm_navigation_msgs.srv import *
from arm_navigation_msgs.msg import *

import threading

class PlanningScene:
    def __init__(self):
	self.scene_diff = SetPlanningSceneDiffRequest()
	self.write_scene = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
	self.lock = threading.Lock()
	self.get_current_scene()
    
    def get_current_scene(self):
	with self.lock:
	    return self.write_scene(self.scene_diff).planning_scene
    
    def set_joint_state(self, names, positions):
	with self.lock:
	    self.scene_diff.planning_scene_diff.robot_state.joint_state.name = list(self.scene_diff.planning_scene_diff.robot_state.joint_state.name)
	    self.scene_diff.planning_scene_diff.robot_state.joint_state.position = list(self.scene_diff.planning_scene_diff.robot_state.joint_state.position)
	    for n,p in zip(names,positions):
		try:
		    index = self.scene_diff.planning_scene_diff.robot_state.joint_state.name.index(n)
		    self.scene_diff.planning_scene_diff.robot_state.joint_state.position[index] = p
		except ValueError:
		    self.scene_diff.planning_scene_diff.robot_state.joint_state.name.append(n)
		    self.scene_diff.planning_scene_diff.robot_state.joint_state.position.append(p)
	    self.write_scene(self.scene_diff)
    def set_robot_state(self, rs):
        self.scene_diff.planning_scene_diff.robot_state = rs
        self.write_scene(self.scene_diff)
