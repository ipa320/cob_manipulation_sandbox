class ErrorCode:
    def __init__(self, msg=None):
        self.error_code = msg
        self.success = msg is None

class MotionPlan:
    def __init__(self, planning_scene = None)
        if planning_scene:
            self.planning_scene = planning_scene
        else
            self.planning_scene = PlanningScene()
         self.executables = []
    def plan(retries = 1):
        for ex in self.executables:
            for i in range(retries+1):
                error_code = ex.plan(self.planning_scene)
                if !error_code.success:
                    return error_code
        return ErrorCode()
    def execute(retries = 1):
        for ex in self.executables:
            for i in range(retries +1):
                error_code = ex.execute()
                if !error_code.success:
                    return error_code
        return ErrorCode()

        
class MoveHand:
    def __init__(self, name, target, verify_cb, blocking):
        self.name = name
        self.target = look_up(name, target)
        self.robot_state = 
        self.verify_cb = verify_cb
        self.blocking = blocking
    def plan(self, planning_scene):
        planning_scene.set_robot_state(build_robot_state(group=target, positons = target))
        return ErrorCode()
    def execute(self):
        error_code = hands[self.name].move(self.target, self.blocking)
        if error_code.success:
            error_code self.verify_cb()
        return error_code
        

class MoveArm:
    def __init__(self, name, target):
        self.name = name
        self.target = look_up(name, target)
        self.plan = None
    def plan(self, planning_scene):
        error_code, plan = arms[self.name].plan(planning_scene, target)
        if error_code.success:
            self.plan = plan
        return error_code
    def execute(self):
        if self.plan is None:
        error_code = self.plan()
        if error_code.success:
            error_code = arms[self.name].move(self.plan.trajectory)
        return error_code

class MoveArmInterpolated(MoveArm):
    def __init__(self, name, target):
        self.name = name
        self.target = look_up(name, target)
        self.plan = False
    def plan(self, planning_scene):
        error_code, plan = arms[self.name].plan_interpolated(planning_scene, target)
        if error_code.success:
            self.plan = plan
        return error_code
        
class AttachObject:
    def __init__(name, graspable_object, grasp_configuration):
        self.name = name
        self.graspable_object = graspable_object
        self.grasp_configuration = grasp_configuration
    def plan(self, planning_scene):
        return planning_scene.attach(self.graspable_object)
    def execute(self):
        return robot.attach_object(name, self.graspable_object, grasp_configuration)      
        
        