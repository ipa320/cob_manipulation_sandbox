#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_manipulation_test')
import rospy

from simple_script_server import *
#from script_utils import *
import tf_conversions.posemath as pm

from cob_object_handler.srv import *
from arm_navigation_msgs.srv import *
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
		
			### add world and static obstacles to collision_objects
			self.AddWorld()
			self.HandleObject("add", "table_ikea")	
			self.HandleObject("add", "milk")
			
			#### just for updating in rviz
			#self.SetPlanningSceneDiff()
			
			rospy.loginfo("Start")		
			### init poses
			handle01 = self.sss.move_planned("arm","folded",False)
			self.sss.move("torso","home",False)
			self.sss.move("sdh","home",False)
			self.sss.move("tray","down")
			handle01.wait()
			
			print "To PreGrasp"
			handle01 = self.sss.move_planned("arm","pregrasp")
			handle01.wait()
			
			self.sss.move("sdh","cylopen")
			
			#milk_state = self.GetModelState("milk_model")
			
			#milk_pose = pm.fromMsg(milk_state.pose)
			#print milk_pose
			
			##ToDo: get from GetModelState or ObjectDetection
			grasp_pos = [-0.1,0,0.20]
			grasp_ori = [0,0,0]
			grasp_joint_state, error_code = self.sss.calculate_ik(['arm_7_link', grasp_pos, grasp_ori])
			print grasp_joint_state.position
			self.sss.wait_for_input()
			print "To Grasp"
			self.sss.move("arm",[list(grasp_joint_state.position)])
			
			self.sss.move("sdh", "cylclosed")
			self.HandleObject("attach","milk")
			self.SetPlanningSceneDiff()
			
			#pick_up_pos = [0.3,0,0]
			#pick_up_ori = [0,0,0]
			#pick_up_pos_joint_state, error_code = self.sss.calculate_ik(['arm_7_link', pick_up_pos, pick_up_ori])
			#print pick_up_pos_joint_state.position
			#self.sss.wait_for_input()
			#self.sss.move("arm",[list(pick_up_pos_joint_state.position)])
			print "To PreGrasp"
			self.sss.move("arm","pregrasp")
			
			#print "To WaveIn"
			#handle01 = self.sss.move_planned("arm","wavein")
			#handle01.wait()
			
			print "To Hold"
			handle01 = self.sss.move_planned("arm","hold")
			handle01.wait()
		
			self.SetPlanningSceneDiff()
			
			print "To Folded"
			handle01 = self.sss.move_planned("arm","folded")
			handle01.wait()
			
			
			put_down_pos = [0.51, -0.29, 1.23]	#[0.537, -0.206, 1.149]	#[0.492, -0.373, 1.316]
			put_down_ori = [0.22, -1.3, -1.92]	#[0.189, -1.401, -1.756]	#[0.255, -1.206, -2.107]
			put_down_pos_joint_state, error_code = self.sss.calculate_ik(['base_footprint', put_down_pos, put_down_ori])
			print put_down_pos_joint_state.position
			#self.sss.wait_for_input()
			#self.sss.move_planned("arm",[list(put_down_pos_joint_state.position)])		
			
			print "To OverTray"
			handle01 = self.sss.move_planned("arm","overtray")
			handle01.wait()
			
			
			self.sss.move("tray","up")
			self.sss.move("sdh","cylopen")
			self.HandleObject("detach","milk")
			self.SetPlanningSceneDiff()
			
			
			#handle01 = self.sss.move("arm",[list(put_down_pos_joint_state.position)],True)
			#handle01.wait()
			put_down_pos = [0.25, 0, 0]
			put_down_ori = [0, 0, 0]
			put_down_pos_joint_state, error_code = self.sss.calculate_ik(['arm_7_link', put_down_pos, put_down_ori])
			print put_down_pos_joint_state.position
			self.sss.wait_for_input()
			print "To Retreat"
			self.sss.move("arm",[list(put_down_pos_joint_state.position)])	
			
			
			self.sss.move("sdh","home")
			
			print "To Folded"
			handle01 = self.sss.move_planned("arm","folded")
			handle01.wait()
			
			
			self.sss.move("sdh","home")
			
			
			
	def SetPlanningSceneDiff(self):
		try:	
			set_planning_scene_diff_req = SetPlanningSceneDiffRequest()
			set_planning_scene_diff_res = SetPlanningSceneDiffResponse()
			set_planning_scene_diff_res = self.set_planning_scene_diff_client(set_planning_scene_diff_req)
			print set_planning_scene_diff_res
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
		return get_model_state_res
					
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
