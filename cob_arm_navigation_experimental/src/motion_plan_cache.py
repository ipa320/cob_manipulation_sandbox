#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_arm_navigation_experimental')
import rospy

from std_srvs.srv import *
from arm_navigation_msgs.srv import *
import threading

class MotionPlanCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.cache = dict()
        rospy.wait_for_service('get_motion_plan')
        self.motion_plan_client = rospy.ServiceProxy('get_motion_plan', GetMotionPlan)
        self.empty_srv = rospy.Service('clear', Empty, self.empty_cache)
        self.cache_plan_srv = rospy.Service('cache_motion_plan', GetMotionPlan, self.cache_motion_plan)
        self.get_plan_srv = rospy.Service('get_cached_motion_plan', GetMotionPlan, self.get_motion_plan)
    def empty_cache(self, req):
        with self.lock:
            self.cache.clear()
    def cache_motion_plan(self, req):
        res = self.motion_plan_client(req)
        if res.error_code.val == res.error_code.SUCCESS:
            hash = str(req.motion_plan_request)
            with self.lock:
                self.cache[hash] = res
        return res
    def get_motion_plan(self, req):
        try:
            hash = str(req.motion_plan_request)
            with self.lock:
                res =  self.cache.pop(hash)
        except KeyError:
            res = self.motion_plan_client(req)
        return res
        
if __name__ == "__main__":
    rospy.init_node('motion_plan_cache')
    motion_plan_cache = MotionPlanCache()
    rospy.loginfo('motion_plan_cache is ready')
    rospy.spin()
