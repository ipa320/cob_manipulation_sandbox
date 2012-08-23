#visualization_tools.py
'''
Functions for creating visualization_msgs.
'''

__docformat__ = "restructuredtext en"

import roslib; roslib.load_manifest('pr2_python')
from visualization_msgs.msg import Marker
import copy
from state_transformer.srv import GetRobotMarker, GetRobotMarkerRequest
import rospy

def marker_at(pose_stamped, ns='', mid=0, mtype=Marker.SPHERE, sx=0.05, sy=0.05, sz=0.05, r=0.0, g=0.0, b=1.0, 
              a=0.8):
    '''
    Returns a single marker at a pose.

    See the visualization_msgs.msg.Marker documentation for more details on any of these fields.

    **Args:**
    
        **pose_stamped (geometry_msgs.msg.PoseStamped):** Pose for marker

        *ns (string):* namespace

        *mid (int):* ID
        
        *mtype (int):* Shape type

        *sx (double):* Scale in x

        *sy (double):* Scale in y

        *sz (double):* scale in z

        *r (double):* Red (scale 0 to 1)

        *g (double):* Green (scale 0 to 1)

        *b (double):* Blue (scale 0 to 1)

        *a (double):* Alpha (scale 0 to 1)

    **Returns:**
        visualization_msgs.msg.Marker at pose_stamped
    '''
    marker = Marker()
    marker.header = copy.deepcopy(pose_stamped.header)
    marker.ns = ns
    marker.id = mid
    marker.type = mtype
    marker.action = marker.ADD
    marker.pose = copy.deepcopy(pose_stamped.pose)
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    return marker

def marker_at_point(point_stamped, ns='', mid=0, mtype=Marker.SPHERE, sx=0.05, sy=0.05, sz=0.05, r=0.0, g=0.0, 
                    b=1.0, a=0.8):
    '''
    Returns a single marker at a point.

    Orientation is always (0, 0, 0, 1).  See the visualization_msgs.msg.Marker documentation for more details on 
    any of these fields.

    **Args:**
    
        **point_stamped (geometry_msgs.msg.PointStamped):** Point for marker

        *ns (string):* namespace

        *mid (int):* ID
        
        *mtype (int):* Shape type

        *sx (double):* Scale in x

        *sy (double):* Scale in y

        *sz (double):* scale in z

        *r (double):* Red (scale 0 to 1)

        *g (double):* Green (scale 0 to 1)

        *b (double):* Blue (scale 0 to 1)

        *a (double):* Alpha (scale 0 to 1)

    **Returns:**
        visualization_msgs.msg.Marker at point_stamped
    '''

    marker = Marker()
    marker.header = copy.deepcopy(point_stamped.header)
    marker.ns = ns
    marker.id = mid
    marker.type = mtype
    marker.action = marker.ADD
    marker.pose.position = copy.deepcopy(point_stamped.point)
    marker.pose.orientation.w = 1.0
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    return marker

def robot_marker(robot_state, link_names=None, ns='', r = 0.0, g = 0.0, b = 1.0, a=0.8, scale=1.0):
    '''
    Returns markers representing the robot.

    To show only the arm or the gripper, use the link_names field with the arm and gripper links filled in.

    **Args:**
    
        **robot_state (arm_navigation_msgs.msg.RobotState):** State at which you want the marker

        *link_names ([string]):* Will return markers for only these links.  If left at None, will return markers
        for the whole robot.

        *ns (string):* Marker namespace.

        *r (double):* Red (scale 0 to 1)
        
        *g (double):* Green (scale 0 to 1)

        *b (double):* Blue (scale 0 to 1)

        *a (double):* Alpha (scale 0 to 1)

        *scale (double):* Scaling for entire robot
        
    **Returns:**
        visualization_msgs.msg.Marker array representing the links or the whole robot at robot_state
    '''
    marker_srv = rospy.ServiceProxy("/get_robot_marker", GetRobotMarker)
    req = GetRobotMarkerRequest()
    req.robot_state = robot_state
    req.link_names = link_names
    if not req.link_names:
        req.link_names = []
    req.ns = ns
    req.color.r = r
    req.color.g = g
    req.color.b = b
    req.color.a = a
    req.scale = scale
    
    res = marker_srv(req)
    return res.marker_array
