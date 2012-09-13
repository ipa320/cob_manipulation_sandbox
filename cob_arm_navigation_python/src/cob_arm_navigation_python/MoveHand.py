import roslib
roslib.load_manifest('cob_arm_navigation_python')

from MotionPlan import *
import simple_script_server
from arm_navigation_msgs.msg import *
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_python.hand_description import HandDescription
from pr2_python.world_interface import WorldInterface

#class MoveHand(MotionExecutable):
#    def __init__(self, arm, target, verify_cb = None):
#        self.name = HandDescription(arm).hand_group
#        self.target = target
#        self.verify_cb = verify_cb
#        self.js = None
#    def plan(self):
#        js = read_target_state_from_param(self.name, self.target)
#        if js: 
#            set_planning_scene_joint_state(js)
#            return ErrorCode()
#        else:
#            return ErrorCode('Lookup for '+str(self.target)+' failed')
#    def execute(self):
#        sss = simple_script_server.simple_script_server()
#        return MotionHandleSSS(sss, (self.name,self.target))
class MoveComponent(MotionExecutable):
    def __init__(self, name, target):
        self.type = "MoveComponent"
        self.name = name
        self.target = target
    def info(self):
        print self.name
        print self.target
    def plan(self):
        js = read_target_state_from_param(self.name, self.target)
        if js: 
            set_planning_scene_joint_state(js)
            return ErrorCode()
        else:
            return ErrorCode('Lookup for '+str(self.target)+' failed')
    def execute(self):
        sss = simple_script_server.simple_script_server()
        return MotionHandleSSS(sss, (self.name,self.target))

class CheckService(MotionExecutable):
    def __init__(self, name, srvtype, args = {}, verify_cb = None):
        self.type = "CheckGrasp"
        self.name = name
        self.srvtype = srvtype
        self.verify_cb = verify_cb
        self.args = args
        self.proxy = rospy.ServiceProxy(name, srvtype)
    def info(self):
        print self.name
        print self.srvtype
        print self.args
    def plan(self):
        try:
            self.proxy.wait_for_service(0.001) # zero is invalid
            return ErrorCode()
        except:
            return ErrorCode('Service '+ self.name +' not available')
    def execute(self):
            try:
                if self.args:
                    res = self.proxy(**self.args)
                else:
                    res = self.proxy()
                ok = True
                if self.verify_cb:
                    ok = self.verify_cb(res)
            except:
                ok = False
            if ok:
                return MotionHandleDummy()
            else:
                return MotionHandleDummy('Check failed')


class AttachObject:
    def __init__(self,arm, object_id):
        self.type = "AttachObject"
        self.arm = arm
        self.object_id = object_id
    def info(self):
		print self.arm
		print self.object_id
    def plan(self):
        get_planning_scene_interface().attach_object_to_gripper(self.arm, self.object_id)
        return ErrorCode()
    def execute(self):
        WorldInterface().attach_object_to_gripper(self.arm, self.object_id)
        return MotionHandleDummy()
class DetachObject:
    def __init__(self,arm, object_id):
        self.type = "DetachObject"
        self.arm = arm
        self.object_id = object_id
    def info(self):
		print self.arm
		print self.object_id
    def plan(self):
        get_planning_scene_interface().detach_and_add_back_attached_object(self.arm, self.object_id)
        return ErrorCode()
    def execute(self):
        WorldInterface().detach_and_add_back_attached_object(self.arm, self.object_id)
        return MotionHandleDummy()
        
class EnableCollision(CollisionOperation):
    def __init__(self, object1, object2):
        self.type = "EnableCollision"
        CollisionOperation.__init__(self)
        self.object1 = object1
        self.object2 = object2
        self.operation = self.DISABLE # disable collision checks
    def info(self):
		print self.object1
		print self.object2
    def plan(self):
        get_planning_scene_interface().add_ordered_collisions(OrderedCollisionOperations([self]))
        return ErrorCode()
    def execute(self):
        return MotionHandleDummy()
    
class ResetCollisions:
    def __init__(self):
        self.type = "ResetCollision"
    def info(self):
        pass
    def plan(self):
        get_planning_scene_interface().set_collisions(None)
        return ErrorCode()
    def execute(self):
        return MotionHandleDummy()
            
     
