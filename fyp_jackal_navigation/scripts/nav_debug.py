#!/usr/bin/env python
"""Navigation black-box recorder + failure verdict.

Run alongside nav_real.launch in its own terminal:
    rosrun fyp_jackal_navigation nav_debug.py

Writes a timestamped log to ~/.ros/nav_debug/ and prints the important
events to the console. When move_base aborts a goal, it prints a VERDICT
naming the most likely cause, based on move_base's actual failure paths
(melodic source):

  "Aborting because a valid control could not be found"
      -> the LOCAL costmap rejected every trajectory. Either the robot is
         pinned by (real or phantom) lethal cells, or it is mislocalized so
         the global path runs through mapped walls.
  "Aborting because a valid plan could not be found"
      -> the GLOBAL costmap has no corridor: stale marks or mislocalization.
  "...robot appears to be oscillating"
      -> planner indecision in a tight spot.

Continuously monitored:
  * AMCL pose covariance        -> localization health (the "return home
                                   fails until I re-set the pose" signature)
  * lethal cells around robot   -> pinned-by-marks detection (local costmap)
  * sensor rates (/scan, /livox/lidar, /mid360/obstacles, odom) -> stalls
  * mid360/obstacles point count -> empty-filter detection
  * recovery events, goals, results, cmd_vel activity

Also raises move_base's internal loggers to DEBUG (param ~debug_loggers),
so per-cycle planner detail lands in ~/.ros/log/latest/move_base*.log.
"""
import datetime
import math
import os

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import LaserScan, PointCloud2

STATUS_TEXT = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED',
               4: 'ABORTED', 5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING',
               8: 'RECALLED', 9: 'LOST'}

# thresholds for the verdict heuristics
LOC_STD_XY_BAD = 0.30      # m
LOC_STD_YAW_BAD = 0.25     # rad (~14 deg)
PIN_RADIUS = 0.35          # m: circumscribed robot radius, "touching" zone
NEAR_RADIUS = 0.60         # m: "boxed in" zone
STALL_SEC = 1.5


class NavDebug(object):
    def __init__(self):
        logdir = os.path.expanduser('~/.ros/nav_debug')
        if not os.path.isdir(logdir):
            os.makedirs(logdir)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(logdir, 'nav_debug_%s.log' % stamp)
        self.f = open(self.path, 'w', 1)  # line-buffered
        self.say('nav_debug recording to %s' % self.path, console=True)

        self.last = {}            # topic -> wall time of last message
        self.amcl_pose = None     # (x, y, yaw, std_xy, std_yaw)
        self.odom_pose = None     # (x, y)
        self.odom_wz = 0.0
        self.goal = None          # (x, y) in map
        self.grid = None          # latest local costmap
        self.cloud_pts = -1
        self.cmd = (0.0, 0.0)
        self.recoveries = []      # wall times of recovery events
        self.active_status = None

        rospy.Subscriber('/rosout_agg', Log, self.on_rosout, queue_size=50)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.on_amcl, queue_size=5)
        rospy.Subscriber('/odometry/filtered', Odometry, self.on_odom,
                         queue_size=5)
        rospy.Subscriber('/move_base/current_goal', PoseStamped,
                         self.on_goal, queue_size=5)
        rospy.Subscriber('/move_base/status', GoalStatusArray,
                         self.on_status, queue_size=5)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid,
                         self.on_grid, queue_size=2)
        rospy.Subscriber('/scan', LaserScan, self.tick('scan'), queue_size=2)
        rospy.Subscriber('/livox/lidar', PointCloud2, self.tick('livox'),
                         queue_size=2)
        rospy.Subscriber('/mid360/obstacles', PointCloud2, self.on_cloud,
                         queue_size=2)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=5)

        if rospy.get_param('~debug_loggers', True):
            self.raise_loggers()

        rospy.Timer(rospy.Duration(2.0), self.snapshot)
        rospy.Timer(rospy.Duration(1.0), self.check_stalls)

    # ------------------------------------------------------------- helpers
    def say(self, text, console=False):
        line = '[%s] %s' % (datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3],
                            text)
        self.f.write(line + '\n')
        if console:
            print(line)

    def tick(self, name):
        def cb(_msg):
            self.last[name] = rospy.get_time()
        return cb

    def raise_loggers(self):
        try:
            from roscpp.srv import SetLoggerLevel
            srv = rospy.ServiceProxy('/move_base/set_logger_level',
                                     SetLoggerLevel)
            srv.wait_for_service(timeout=5.0)
            for logger in ('ros.move_base', 'ros.base_local_planner',
                           'ros.navfn'):
                try:
                    srv(logger=logger, level='debug')
                except Exception:
                    pass
            self.say('move_base loggers raised to DEBUG '
                     '(details in ~/.ros/log/latest/move_base*.log)',
                     console=True)
        except Exception as e:
            self.say('could not raise move_base loggers: %s' % e, console=True)

    # ----------------------------------------------------------- callbacks
    def on_rosout(self, msg):
        if msg.name not in ('/move_base', '/amcl'):
            return
        if msg.level < Log.WARN:
            return
        self.say('%s %s: %s' % ('WARN' if msg.level == Log.WARN else 'ERROR',
                                msg.name, msg.msg), console=True)
        text = msg.msg
        if 'recovery' in text.lower() or 'Clearing' in text:
            self.recoveries.append(rospy.get_time())
        if 'Aborting' in text:
            self.verdict(text)

    def on_amcl(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c = msg.pose.covariance
        std_xy = math.sqrt(max(c[0], c[7]))
        std_yaw = math.sqrt(abs(c[35]))
        self.amcl_pose = (p.x, p.y, yaw, std_xy, std_yaw)
        self.last['amcl'] = rospy.get_time()
        if std_xy > LOC_STD_XY_BAD or std_yaw > LOC_STD_YAW_BAD:
            self.say('LOCALIZATION DEGRADED: std_xy=%.2f m std_yaw=%.2f rad'
                     % (std_xy, std_yaw), console=True)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pose = (p.x, p.y)
        self.odom_wz = msg.twist.twist.angular.z
        self.last['odom'] = rospy.get_time()

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.say('NEW GOAL: (%.2f, %.2f) in %s'
                 % (self.goal[0], self.goal[1], msg.header.frame_id),
                 console=True)

    def on_status(self, msg):
        for s in msg.status_list:
            if s.status != self.active_status:
                self.active_status = s.status
                self.say('move_base status -> %s (%s)'
                         % (STATUS_TEXT.get(s.status, s.status), s.text),
                         console=(s.status in (3, 4, 5)))

    def on_grid(self, msg):
        self.grid = msg
        self.last['costmap'] = rospy.get_time()

    def on_cloud(self, msg):
        self.cloud_pts = msg.width
        self.last['mid360'] = rospy.get_time()

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    # ------------------------------------------------------------ analysis
    def pin_metrics(self):
        """(lethal cells within PIN_RADIUS, within NEAR_RADIUS) of robot."""
        if self.grid is None or self.odom_pose is None:
            return None
        g = self.grid
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = np.asarray(g.data, dtype=np.int16).reshape(g.info.height,
                                                          g.info.width)
        rx, ry = self.odom_pose
        cx = int((rx - ox) / res)
        cy = int((ry - oy) / res)
        out = []
        for radius in (PIN_RADIUS, NEAR_RADIUS):
            r = int(radius / res) + 1
            x0, x1 = max(cx - r, 0), min(cx + r + 1, g.info.width)
            y0, y1 = max(cy - r, 0), min(cy + r + 1, g.info.height)
            if x0 >= x1 or y0 >= y1:
                out.append(-1)
                continue
            win = data[y0:y1, x0:x1]
            yy, xx = np.mgrid[y0:y1, x0:x1]
            d2 = (xx - cx) ** 2 + (yy - cy) ** 2
            mask = d2 <= (radius / res) ** 2
            out.append(int(((win >= 99) & mask).sum()))
        return out

    def snapshot(self, _evt):
        parts = []
        if self.amcl_pose:
            x, y, yaw, sxy, syaw = self.amcl_pose
            parts.append('pose=(%.2f,%.2f,%.0fdeg) loc_std=(%.2fm,%.2frad)'
                         % (x, y, math.degrees(yaw), sxy, syaw))
        if self.goal and self.amcl_pose:
            d = math.hypot(self.goal[0] - self.amcl_pose[0],
                           self.goal[1] - self.amcl_pose[1])
            parts.append('goal_dist=%.2f' % d)
        pins = self.pin_metrics()
        if pins:
            parts.append('lethal_cells(<%.2gm)=%d (<%.2gm)=%d'
                         % (PIN_RADIUS, pins[0], NEAR_RADIUS, pins[1]))
        parts.append('mid360_pts=%d' % self.cloud_pts)
        parts.append('cmd=(%.2f,%.2f)' % self.cmd)
        parts.append('wz=%.2f' % self.odom_wz)
        self.say('STATUS ' + ' '.join(parts))

    def check_stalls(self, _evt):
        now = rospy.get_time()
        for name in ('scan', 'livox', 'mid360', 'odom'):
            t = self.last.get(name)
            if t is not None and now - t > STALL_SEC:
                self.say('SENSOR STALL: %s silent for %.1f s'
                         % (name, now - t), console=True)

    def verdict(self, abort_text):
        now = rospy.get_time()
        recent_recoveries = sum(1 for t in self.recoveries if now - t < 60.0)
        pins = self.pin_metrics() or [-1, -1]
        lines = ['', '=========================== VERDICT ===========================',
                 'move_base said: %s' % abort_text,
                 'recoveries in last 60 s: %d' % recent_recoveries,
                 'lethal cells within %.2f m: %d, within %.2f m: %d'
                 % (PIN_RADIUS, pins[0], NEAR_RADIUS, pins[1]),
                 'mid360/obstacles points: %d' % self.cloud_pts]
        causes = []
        if self.amcl_pose:
            _, _, _, sxy, syaw = self.amcl_pose
            lines.append('localization std: %.2f m, %.2f rad' % (sxy, syaw))
            if sxy > LOC_STD_XY_BAD or syaw > LOC_STD_YAW_BAD:
                causes.append('MISLOCALIZED (std high). In RViz check the '
                              'orange /scan vs the map walls; fix with a '
                              'fresh 2D Pose Estimate. Do NOT restart.')
        if pins[0] > 0:
            causes.append('PINNED: lethal cells inside the robot footprint '
                          'zone. Check the magenta /mid360/obstacles and the '
                          'local costmap: real object touching, or phantom '
                          'marks (screenshot it).')
        elif pins[1] > 8:
            causes.append('BOXED IN: heavy lethal marking within %.2f m. '
                          'Tight gap + inflation, or phantom clutter.'
                          % NEAR_RADIUS)
        stalled = [n for n in ('scan', 'livox', 'mid360', 'odom')
                   if self.last.get(n) and now - self.last[n] > STALL_SEC]
        if stalled:
            causes.append('SENSOR STALL: %s not publishing.' % ', '.join(stalled))
        if not causes:
            causes.append('No single smoking gun: likely a tight-gap '
                          'maneuver failure (goal needs sharp turn near '
                          'obstacles). Try a goal in open space, approach '
                          'the gap head-on.')
        lines.append('most likely cause(s):')
        for c in causes:
            lines.append('  * ' + c)
        lines.append('full log: %s' % self.path)
        lines.append('move_base internals: ~/.ros/log/latest/move_base*.log')
        lines.append('===============================================================')
        for ln in lines:
            self.say(ln, console=True)


if __name__ == '__main__':
    rospy.init_node('nav_debug')
    NavDebug()
    rospy.spin()
