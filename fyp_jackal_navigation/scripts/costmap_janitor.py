#!/usr/bin/env python
"""Periodically clear move_base's costmaps.

Live marks are painted into the global costmap through AMCL's pose at the
moment of observation; AMCL jitter makes the same obstacle edge land a few
cm differently each time, so obstacles slowly 'fatten' over a run until a
narrow corridor seals shut - the planner then detours or fails outright.

Wiping the costmaps every few seconds resets that accumulation. It is safe
in THIS system because obstacle memory lives in the mid360_filter node
(1.5 s ego-motion-compensated window, republished at 10 Hz) and in the
40 Hz Hokuyo - everything real is re-marked within ~0.1 s of a wipe.
"""
import rospy
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node('costmap_janitor')
    period = rospy.get_param('~period', 10.0)
    if period <= 0:
        rospy.loginfo('costmap_janitor disabled (period <= 0)')
        rospy.spin()
    else:
        rospy.wait_for_service('/move_base/clear_costmaps')
        clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo('costmap_janitor: clearing costmaps every %.1f s', period)
        rate = rospy.Rate(1.0 / period)
        while not rospy.is_shutdown():
            try:
                clear()
            except rospy.ServiceException:
                pass
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
