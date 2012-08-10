#hand_description.py
'''
Description of a robot end effector.
'''

__docformat__ = "restructuredtext en"

import roslib; roslib.load_manifest('cob_arm_navigation_python')
import rospy
import numpy as np

#change these as ros parameters rather than changing them here

class HandDescription:
    '''
    This class reads in the hand description file and stores useful information about a robot's arm and hand.
    
    A hand has fingertips, fingers, and a palm.

    **Attributes:**
    
        **arm_name (string):** The name of the arm to which this hand belongs

        **hand_group (string):** The name of the group describing all hand links

        **attach_link (string):** The link that remains stationary with respect to attached objects

        **hand_links ([string]):** The links that make up the hand

        **touch_links ([string]):** The links that touch the object when attached

        **finger_tip_links ([string]):** The names of the links corresponding to the fingertips

        **approach_direction ([x, y, z] normalized):** In the hand's frame, the long direction of the gripper.
  
        **hand_frame (string):** The name of the frame that controls the hand.  This is the frame for which planning
        is done.

        **gripper_joint (string):** The name of the joint controlling the gripper.

        **end_effector_length (double):** The length (in m) of the end effector from the hand_frame to the tip.
    '''
    def __init__(self, arm_name):
        self.arm_name = arm_name
        self.hand_group = rospy.get_param('/hand_description/'+arm_name+'/hand_group_name', 
                                          "sdh")
        self.attach_link = rospy.get_param('/hand_description/'+arm_name+'/attach_link',
                                           "sdh_palm_link")
        self.touch_links = rospy.get_param('/hand_description/'+arm_name+'/hand_touch_links',
                                           ['sdh_palm_link','sdh_finger_11_link','sdh_finger_12_link','sdh_finger_13_link','sdh_finger_21_link','sdh_finger_22_link','sdh_finger_23_link','sdh_thumb_1_link','sdh_thumb_2_link','sdh_thumb_3_link'])
