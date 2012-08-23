from MotionPlan import *

def grasp_object(arm_name, hand_name, graspable_object):
    error_code = ErrorCode('grasp_not_available')
    gp = GraspPlannerInterface()
    for gc in gp.get_configurations(arm_name, hand_name, graspable_object)
    
        mp = MotionPlan()

        # mp += MoveBase(gc.location)
        
        mp += MoveHand(hand_name, gc.hand_open, blocking=False)
        
        mp += MoveArm(arm_name, gc.arm_pose) #Planned with open hand, in object link
        
        mp += MoveHand(hand_name, gc.hand_grasp, verify_cb = gc.verify) # verification is done in executing stage only
        
        mp += AttachObject(hand_name, graspable_object, gc)
        
        mp += MoveArmInterpolated(arm_name ,gc.arm_lift) #Planned with object
        
        mp += MoveArm(arm_name,'hold') #Planned with object
        
        error_code = mp.plan(retries = 5)
        
        if error_code.success: # planned all steps
            error_code = mp.excute(retries = 2):
            break # error handling is not done here
        else:
            gc.reject(error_code)
    return error_code       
    
