import roslib
roslib.load_manifest('cob_arm_navigation_python')

import rospy
from MotionPlan import *
from MoveArm import *
from MoveHand import *

rospy.init_node("test")

mp = MotionPlan()
mp += MoveArm('arm', 'folded')
mp += MoveArm('arm', 'pregrasp')
mp += MoveHand('sdh', 'cylopen')
mp += MoveArm('arm', ['arm_7_link',[0,0,0.1]])
mp += MoveHand('sdh', 'cylclosed')
mp += MoveArm('arm', 'hold')

mp.plan()
for e in mp.execute():
    print e.wait()
