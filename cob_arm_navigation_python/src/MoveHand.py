import roslib
roslib.load_manifest('cob_arm_navigation_python')

from MotionPlan import *
import simple_script_server
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
