#!/usr/bin/env python
"""Point-to-point waypoint navigation with localization warmup.

Waypoints (x, y, yaw in the map frame) live in
fyp_jackal_navigation/config/waypoints.yaml.

Record a waypoint (drive/teleop the robot to the spot first):
    rosrun fyp_jackal_navigation waypoint_nav.py --record home
    rosrun fyp_jackal_navigation waypoint_nav.py --record pointA

Show the robot's current position and heading:
    rosrun fyp_jackal_navigation waypoint_nav.py --where

Navigate (nav_real.launch must be running; robot roughly localized):
    rosrun fyp_jackal_navigation waypoint_nav.py pointA
    rosrun fyp_jackal_navigation waypoint_nav.py home
    rosrun fyp_jackal_navigation waypoint_nav.py pointA home   # multi-leg

Before the first leg the robot creeps ~0.2 m forward (clear space ahead
required!) and waits for AMCL to converge, then sends each waypoint to
move_base in turn. An aborted goal gets one retry after clearing costmaps.
"""
import argparse
import math
import os
import sys

import actionlib
import rospkg
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def default_file():
    pkg = rospkg.RosPack().get_path('fyp_jackal_navigation')
    return os.path.join(pkg, 'config', 'waypoints.yaml')


class WaypointNav(object):
    def __init__(self):
        self.pose = None        # (x, y, yaw)
        self.std = (99.0, 99.0)  # (xy, yaw)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.on_pose, queue_size=2)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

    def on_pose(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))
        c = msg.pose.covariance
        self.std = (math.sqrt(max(c[0], c[7])), math.sqrt(abs(c[35])))

    def wait_pose(self, timeout=10.0):
        t0 = rospy.get_time()
        while self.pose is None and rospy.get_time() - t0 < timeout \
                and not rospy.is_shutdown():
            rospy.sleep(0.1)
        return self.pose is not None

    def where(self):
        if not self.wait_pose():
            print('No /amcl_pose received - is nav_real.launch running?')
            return 1
        x, y, yaw = self.pose
        print('position: x=%.3f  y=%.3f   (map frame)' % (x, y))
        print('heading : %.3f rad = %.1f deg  (0 deg = map x-axis, CCW+)'
              % (yaw, math.degrees(yaw)))
        print('loc std : %.2f m, %.2f rad' % self.std)
        return 0

    def record(self, name, path):
        if not self.wait_pose():
            print('No /amcl_pose - cannot record. Is navigation running?')
            return 1
        data = {}
        if os.path.isfile(path):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        x, y, yaw = self.pose
        data[name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        with open(path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=True)
        print('recorded "%s": x=%.3f y=%.3f yaw=%.1f deg  ->  %s'
              % (name, x, y, math.degrees(yaw), path))
        return 0

    def warmup(self, dist=0.2, speed=0.1):
        print('warmup: creeping %.2f m forward for localization '
              '(clear space ahead!)' % dist)
        tw = Twist()
        tw.linear.x = speed
        rate = rospy.Rate(10)
        t_end = rospy.get_time() + dist / speed
        while rospy.get_time() < t_end and not rospy.is_shutdown():
            self.cmd_pub.publish(tw)
            rate.sleep()
        self.cmd_pub.publish(Twist())  # stop
        # wait for AMCL to settle
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < 8.0 and not rospy.is_shutdown():
            if self.std[0] < 0.20 and self.std[1] < 0.15:
                print('localization converged (std %.2f m, %.2f rad)' % self.std)
                return
            rospy.sleep(0.2)
        print('WARNING: localization still loose (std %.2f m, %.2f rad) - '
              'continuing anyway' % self.std)

    def goto(self, name, wp, client):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp['x']
        goal.target_pose.pose.position.y = wp['y']
        yaw = wp.get('yaw', 0.0)
        goal.target_pose.pose.orientation = Quaternion(
            0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        for attempt in (1, 2):
            print('-> going to "%s" (x=%.2f y=%.2f yaw=%.0f deg), attempt %d'
                  % (name, wp['x'], wp['y'], math.degrees(yaw), attempt))
            client.send_goal(goal)
            client.wait_for_result()
            if client.get_state() == GoalStatus.SUCCEEDED:
                print('   reached "%s"' % name)
                return True
            print('   attempt %d failed (%s)'
                  % (attempt, client.get_goal_status_text()))
            if attempt == 1:
                try:
                    rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
                    rospy.sleep(1.0)
                except rospy.ServiceException:
                    pass
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Waypoint navigation with localization warmup')
    parser.add_argument('waypoints', nargs='*',
                        help='waypoint names to visit in order')
    parser.add_argument('--record', metavar='NAME',
                        help='save the current pose under this name')
    parser.add_argument('--where', action='store_true',
                        help='print current position and heading')
    parser.add_argument('--file', default=None, help='waypoints yaml path')
    parser.add_argument('--no-warmup', action='store_true',
                        help='skip the forward creep before the first leg')
    parser.add_argument('--warmup-dist', type=float, default=0.2)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('waypoint_nav')
    nav = WaypointNav()
    path = args.file or default_file()

    if args.where:
        sys.exit(nav.where())
    if args.record:
        sys.exit(nav.record(args.record, path))
    if not args.waypoints:
        parser.print_help()
        sys.exit(1)

    if not os.path.isfile(path):
        print('No waypoint file at %s - record some first.' % path)
        sys.exit(1)
    with open(path) as f:
        table = yaml.safe_load(f) or {}
    missing = [w for w in args.waypoints if w not in table]
    if missing:
        print('Unknown waypoint(s): %s. Known: %s'
              % (', '.join(missing), ', '.join(sorted(table))))
        sys.exit(1)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print('waiting for move_base...')
    client.wait_for_server()

    if not args.no_warmup:
        nav.wait_pose()
        nav.warmup(dist=args.warmup_dist)

    for name in args.waypoints:
        if not nav.goto(name, table[name], client):
            print('FAILED at "%s" - stopping the sequence.' % name)
            sys.exit(2)
    print('all waypoints reached.')


if __name__ == '__main__':
    main()
