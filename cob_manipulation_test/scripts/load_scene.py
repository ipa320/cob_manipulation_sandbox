#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_manipulation_test')
import rospy

from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *

def createObject(shapetype, name, pos, euler, extent):
    co = CollisionObject()
    s = Shape()
    co.id = name
    co.operation.operation = CollisionObjectOperation.ADD
    co.header.frame_id = "/odom_combined";
    co.header.stamp = rospy.Time.now();
    s.type = shapetype
    s.dimensions = list(extent)
    co.shapes = [s]
    p = Pose()
    p.position.x, p.position.y, p.position.z = pos
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_euler(*euler)
    co.poses = [p]
    print co
    return co
   
if __name__ == '__main__':
    rospy.init_node('load_scene')
    while rospy.get_time() == 0.0: pass
    pub = rospy.Publisher('/collision_object', CollisionObject)
    rospy.sleep(1.0)
    pub.publish( createObject(Shape.BOX, "table_ikea", [-0.9,0,0.4],[0,0,0],[0.4, 0.8, 0.8]) )
    rospy.sleep(1.0)
    pub.publish( createObject(Shape.CYLINDER, "milk", [-0.8,-0.2,0.9],[0,0,0],[0.05, 0.2]) )
    rospy.sleep(1.0)
    print rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)(SetPlanningSceneDiffRequest())
