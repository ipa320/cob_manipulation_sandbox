#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_manipulation_test')
import rospy

from simple_script_server import *
#from script_utils import *

from cob_object_handler.srv import HandleObject
from arm_navigation_msgs.srv import SetPlanningSceneDiff
from gazebo_msgs.srv import *

SET_PLANNING_SCENE_DIFF_NAME = '/environment_server/set_planning_scene_diff'

class Test_Object_Handler(script):
	def Initialize(self):
		#self.sss.init("tray")
		#self.sss.init("torso")
		#self.sss.init("arm")
		#self.sss.init("sdh")
		#self.sss.set_light("red")
		
		
		rospy.wait_for_service(SET_PLANNING_SCENE_DIFF_NAME)
		self.set_planning_scene_diff_client = rospy.ServiceProxy(SET_PLANNING_SCENE_DIFF_NAME, SetPlanningSceneDiff)
		
		rospy.wait_for_service('/addWorld')
		self.add_world_client = rospy.ServiceProxy('addWorld', HandleObject)
		
		rospy.wait_for_service('object_handler/handle_object')
		self.handle_object_client = rospy.ServiceProxy('/object_handler/handle_object', HandleObject)
		
		rospy.wait_for_service('/gazebo/get_model_state')
		self.get_model_state_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		
	
	
	def Run(self):
		if not self.sss.parse:
			rospy.loginfo("Testing Object_Handler modes...")
		
		rospy.loginfo("Start")		
		### init poses
		#handle01 = self.sss.move("arm","folded",False)
		#self.sss.move("torso","home",False)
		#self.sss.move("sdh","home",False)
		#self.sss.move("tray","down")
		#handle01.wait()
		
		self.GetModelState("milk_model")
		
		#self.HandleObject("add", "table_ikea")		
		#self.AddWorld()	
		#SetPlanningSceneDiff()	
			
			
	def SetPlanningSceneDiff(self):
		try:	
			set_planning_scene_diff_req = SetPlanningSceneDiffRequest()
			set_planning_scene_diff_res = SetPlanningSceneDiffResponse()
			set_planning_scene_diff_res = self.set_planning_scene_diff_client(set_planning_scene_diff_req)
			print set_planning_diff_res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e		

	def GetModelState(self, model_name):
		try:
			get_model_state_req = GetModelStateRequest()
			get_model_state_req.model_name = model_name
			get_model_state_res = self.get_model_state_client(get_model_state_req)
			print get_model_state_res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
					
	def AddWorld(self):
		rospy.loginfo("AddWorld")
		try:	
			addWorld_req = HandleObjectRequest()
			addWorld_req.operation.data = "add"
			addWorld_res = HandleObjectResponse()
			addWorld_res = self.add_world_client(addWorld_req)
			print addWorld_res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e				
			
	def HandleObject(self, operation, object_name):
		rospy.loginfo("HandleObject")
		try:	
			handle_object_req = HandleObjectRequest()
			handle_object_req.operation.data = operation
			handle_object_req.object.data = object_name
			handle_object_res = HandleObjectResponse()
			handle_object_res = self.handle_object_client(handle_object_req)
			print handle_object_res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e				
			
		
		
		
		

if __name__ == "__main__":
	SCRIPT = Test_Object_Handler()
	SCRIPT.Start()
