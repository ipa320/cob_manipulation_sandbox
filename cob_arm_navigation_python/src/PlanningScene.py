import roslib
roslib.load_manifest('cob_arm_navigation_python')
import rospy

from arm_navigation_msgs.srv import *
from arm_navigation_msgs.msg import *
from geometry_msgs.msg import *
from copy import deepcopy
from tf.transformations import *
from pr2_python.transform_state import TransformState

import threading


class PlanningScene:
    def __init__(self):
	self.scene_diff = SetPlanningSceneDiffRequest()
	self.write_scene = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
	self.get_scene = rospy.ServiceProxy('/environment_server/get_planning_scene', GetPlanningScene)
	self.lock = threading.Lock()
	self.write_current_scene()
	self.transformer = TransformState()

    def get_current_scene(self):
	with self.lock:
	    return self.get_scene(GetPlanningSceneRequest(self.scene_diff.planning_scene_diff,self.scene_diff.operations)).planning_scene
    def write_current_scene(self):
	with self.lock:
	    return self.write_scene(self.scene_diff).planning_scene
	rospy.sleep(1.0)
    def get_robot_state(self):
	return self.get_current_scene().robot_state
    def transform_pose(self, new_frame_id, old_frame_id, pose):
	return self.transformer.transform_pose(new_frame_id, old_frame_id, pose, self.get_robot_state())
	
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
	
    def get_collision_object(self, object_id,scene = None):
	if scene is None:
	    scene = self.get_current_scene()
	obj = None
	for c in scene.collision_objects:
	    if c.id == object_id:
		obj = c
		break
	#print scene
	return obj
    def attach_and_remove_object(self, hand, object_id):
	obj = self.get_collision_object(object_id)
	if obj is None:
	    return False
	obj.operation.operation = obj.operation.REMOVE
	with self.lock:
	    self.scene_diff.planning_scene_diff.collision_objects.append(obj)
	aobj = AttachedCollisionObject()
	aobj.link_name = "sdh_palm_link"
	aobj.object = deepcopy(obj)
	aobj.object.operation.operation = aobj.object.operation.ADD
	aobj.touch_links = ['sdh_palm_link','sdh_finger_11_link','sdh_finger_12_link','sdh_finger_13_link','sdh_finger_21_link','sdh_finger_22_link','sdh_finger_23_link','sdh_thumb_1_link','sdh_thumb_2_link','sdh_thumb_3_link']
	for i in range(len(aobj.object.poses)):
	    aobj.object.poses[i] = self.transform_pose("sdh_palm_link",aobj.object.header.frame_id,aobj.object.poses[i])
	aobj.object.header.frame_id = "sdh_palm_link"
	with self.lock:
	    self.scene_diff.planning_scene_diff.attached_collision_objects.append(aobj)
	self.write_current_scene()
	return True
    def reset_collisions(self):
	with self.lock:
	    self.scene_diff.operations = OrderedCollisionOperations()
	self.write_current_scene()
    def enable_collision(self,object1, object2):
	op = CollisionOperation()
	op.object1 = object1
	op.object2 = object2
	op.operation = op.DISABLE
	with self.lock:
	    self.scene_diff.operations.collision_operations.append(op)
	print self.write_current_scene()



