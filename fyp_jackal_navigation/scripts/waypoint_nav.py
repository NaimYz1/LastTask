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

    def set_pose(self, name, wp):
        """Re-initialize AMCL at a waypoint (robot must physically be there)."""
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        yaw = wp.get('yaw', 0.0)
        msg.pose.pose.position.x = wp['x']
        msg.pose.pose.position.y = wp['y']
        msg.pose.pose.orientation = Quaternion(
            0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        # Tighter than RViz's 2D Pose Estimate: a waypoint pose means the
        # robot is physically ON the marker (within ~15 cm / ~10 deg), not
        # merely "somewhere around here". The home area has few mapped
        # features, so with a loose 0.5 m seed AMCL could not converge
        # within the short warmup creep.
        msg.pose.covariance[0] = 0.03    # std ~0.17 m
        msg.pose.covariance[7] = 0.03
        msg.pose.covariance[35] = 0.02   # std ~8 deg
        # publish a few times so amcl definitely receives it, even if it
        # starts after us
        for _ in range(5):
            if rospy.is_shutdown():
                break
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rospy.sleep(1.0)
        print('AMCL pose set to "%s": x=%.2f y=%.2f yaw=%.0f deg'
              % (name, wp['x'], wp['y'], math.degrees(yaw)))
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

    def warmup(self, dist=0.2, speed=0.1, force=False):
        # already tight (e.g. right after --from or the launch auto-init)?
        # skip the creep entirely.
        rospy.sleep(0.5)
        if self.std[0] < 0.25 and self.std[1] < 0.18:
            print('localization already tight (std %.2f m, %.2f rad) - '
                  'skipping warmup' % self.std)
            return True
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
                return True
            rospy.sleep(0.2)
        if self.std[0] > 0.35 and not force:
            print('ABORT: localization too loose to navigate safely '
                  '(std %.2f m, %.2f rad).' % self.std)
            print('Is the robot really where AMCL thinks it is? If it is '
                  'standing on a waypoint, rerun with:  --from <that waypoint>')
            print('(or override with --force)')
            return False
        print('WARNING: localization loose (std %.2f m, %.2f rad) - '
              'continuing (%s)' % (self.std + ('--force',) if force else
                                   self.std + ('below abort threshold',)))
        return True

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
    parser.add_argument('--set-pose', metavar='NAME', dest='set_pose',
                        help='re-initialize AMCL at this waypoint '
                             '(robot must physically be standing there)')
    parser.add_argument('--from', metavar='NAME', dest='from_wp',
                        help='the waypoint the robot is physically standing '
                             'on right now: its saved pose is applied to '
                             'AMCL before navigating')
    parser.add_argument('--force', action='store_true',
                        help='navigate even if localization stays loose '
                             'after warmup')
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
    if args.set_pose:
        if not os.path.isfile(path):
            print('No waypoint file at %s' % path)
            sys.exit(1)
        with open(path) as f:
            table = yaml.safe_load(f) or {}
        if args.set_pose not in table:
            print('Unknown waypoint "%s". Known: %s'
                  % (args.set_pose, ', '.join(sorted(table))))
            sys.exit(1)
        sys.exit(nav.set_pose(args.set_pose, table[args.set_pose]))
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

    if args.from_wp:
        if args.from_wp not in table:
            print('Unknown --from waypoint "%s". Known: %s'
                  % (args.from_wp, ', '.join(sorted(table))))
            sys.exit(1)
        wp = table[args.from_wp]
        nav.wait_pose()
        # If AMCL is already tight AND roughly agrees with the waypoint
        # (e.g. the robot NAVIGATED here - it stops anywhere within the
        # goal tolerance of the marker), keep AMCL's own estimate: it is
        # more precise than asserting the marker pose, and overwriting a
        # correct belief with an approximate one misaligns the whole map.
        # Only apply the reset when AMCL is loose or clearly disagrees
        # (i.e. the robot was manually placed / is lost).
        d = dyaw = None
        if nav.pose is not None:
            d = math.hypot(nav.pose[0] - wp['x'], nav.pose[1] - wp['y'])
            raw = nav.pose[2] - wp.get('yaw', 0.0)
            dyaw = abs(math.atan2(math.sin(raw), math.cos(raw)))
        if (d is not None and nav.std[0] < 0.25 and d < 0.5
                and dyaw < math.radians(35)):
            print('AMCL already tight and agrees with "%s" (off by %.2f m, '
                  '%.0f deg) - keeping its own estimate'
                  % (args.from_wp, d, math.degrees(dyaw)))
        else:
            print('applying saved pose of "%s" to AMCL...' % args.from_wp)
            nav.set_pose(args.from_wp, wp)

    if not args.no_warmup:
        nav.wait_pose()
        if not nav.warmup(dist=args.warmup_dist, force=args.force):
            sys.exit(3)

    for name in args.waypoints:
        if not nav.goto(name, table[name], client):
            print('FAILED at "%s" - stopping the sequence.' % name)
            sys.exit(2)
    print('all waypoints reached.')


if __name__ == '__main__':
    main()
