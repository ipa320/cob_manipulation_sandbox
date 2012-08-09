import roslib
roslib.load_manifest('cob_arm_navigation_python')

from MotionPlan import *
import simple_script_server
from arm_navigation_msgs.msg import *
import PlanningScene

class MoveHand(MotionExecutable):
    def __init__(self, name, target, verify_cb = None, blocking = False):
        self.name = name
        self.target = target
        self.verify_cb = verify_cb
        self.blocking = blocking
        self.js = None
    def plan(self, planning_scene=None):
        if planning_scene == None:
            planning_scene = PlanningScene.PlanningScene()
        js = read_target_state_from_param(self.name, self.target)
        if js: 
            planning_scene.set_joint_state(js.name, js.position)
            return ErrorCode()
        else:
            return ErrorCode('Lookup for '+str(self.target)+' failed')
    def execute(self):
        sss = simple_script_server.simple_script_server()
        return MotionHandleSSS(sss, (self.name,self.target))

"""class AttachObject:
    def __init__(name, graspable_object, grasp_configuration):
        self.name = name
        self.graspable_object = graspable_object
        self.grasp_configuration = grasp_configuration
    def plan(self, planning_scene):
        return planning_scene.attach(self.graspable_object)
    def execute(self):
        return robot.attach_object(name, self.graspable_object, grasp_configuration)      
"""
class AttachObject:
    def __init__(self,hand, object_id):
        self.hand = hand
        self.object_id = object_id
        self._attached_object_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)
        self._object_pub = rospy.Publisher('collision_object', CollisionObject)
    def plan(self, planning_scene = None):
        if planning_scene == None:
            planning_scene = PlanningScene.PlanningScene()
        #print "CURRENT", planning_scene.get_current_scene()
        if not planning_scene.attach_and_remove_object(self.hand, self.object_id):
            return ErrorCode("Collision object " + self.object_id + " does not exist")
        return ErrorCode()
    def execute(self):
        planning_scene = PlanningScene.PlanningScene()
        aobj = AttachedCollisionObject()
        aobj.link_name = "sdh_palm_link"
        aobj.object = planning_scene.get_collision_object(self.object_id)
        aobj.object.operation.operation = aobj.object.operation.ATTACH_AND_REMOVE_AS_OBJECT
        aobj.touch_links = ['sdh_palm_link','sdh_finger_11_link','sdh_finger_12_link','sdh_finger_13_link','sdh_finger_21_link','sdh_finger_22_link','sdh_finger_23_link','sdh_thumb_1_link','sdh_thumb_2_link','sdh_thumb_3_link']
        self._attached_object_pub.publish(aobj)
        rospy.sleep(0.5)
        return MotionHandleDummy()
class EnableCollision:
    def __init__(self, object1, object2):
        self.object1 = object1
        self.object2 = object2
    def plan(self, planning_scene = None):
        if planning_scene == None:
            planning_scene = PlanningScene.PlanningScene()
        planning_scene.enable_collision(self.object1, self.object2)
        #print planning_scene.get_current_scene()
        return ErrorCode()
    def execute(self):
        planning_scene = PlanningScene.PlanningScene()
        planning_scene.enable_collision(self.object1, self.object2)
        return MotionHandleDummy()
    
class ResetCollisions:
    def plan(self, planning_scene = None):
        if planning_scene == None:
            planning_scene = PlanningScene.PlanningScene()
        planning_scene.reset_collisions()
        return ErrorCode()
    def execute(self):
        planning_scene = PlanningScene.PlanningScene()
        planning_scene.reset_collisions()
        return MotionHandleDummy()
            
     
