#!/usr/bin/env python
"""Mid-360 obstacle filter (TF-free): gravity-aligned height band +
ego-motion-compensated accumulation.

Pipeline per cloud (/livox/lidar, PointCloud2, sensor frame):
  1. fixed mount transform sensor -> base_link (parameters, not TF - another
     system on this robot fights over the livox_frame TF)
  2. height band along WORLD-UP (gravity), not the robot's z-axis: the
     EKF odometry's roll/pitch is used, so when the robot pitches while
     braking or crossing floor tape, the floor 3-4 m ahead does NOT rise
     into the band and create phantom obstacles
  3. accumulate the last ~1.5 s of banded points, each cloud paired with
     the odometry pose at ITS OWN timestamp (so recovery spins do not smear
     points), all republished in the current base_link frame

Thin obstacles (tripod legs, stool poles) stay solidly marked because every
costmap update re-marks them from the whole window; raytrace clearing can
no longer flicker them away.

Band heights are relative to base_link origin (~0.065 m above the floor):
  min_z 0.05 -> keep from ~0.12 m above the floor (sees a stool star-base)
  max_z 0.50 -> ignore above ~0.57 m (robot ~0.50 m tall; the 0.75 m bridge
                deck stays above the band)
"""
import bisect
import collections

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

# PointField datatype -> numpy dtype
_FMT = {PointField.INT8: np.int8, PointField.UINT8: np.uint8,
        PointField.INT16: np.int16, PointField.UINT16: np.uint16,
        PointField.INT32: np.int32, PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}


def quat_to_rot(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([[1.0 - (yy + zz), xy - wz, xz + wy],
                     [xy + wz, 1.0 - (xx + zz), yz - wx],
                     [xz - wy, yz + wx, 1.0 - (xx + yy)]])


class Mid360Filter(object):
    def __init__(self):
        mount_x = rospy.get_param('~mount_x', 0.179)
        mount_y = rospy.get_param('~mount_y', 0.0)
        mount_z = rospy.get_param('~mount_z', 0.375)
        pitch = rospy.get_param('~mount_pitch', 0.6759)  # rad nose-down, measured (38.73 deg)
        self.min_z = rospy.get_param('~min_z', 0.05)     # along world-up, from base origin
        self.max_z = rospy.get_param('~max_z', 0.50)
        self.min_range = rospy.get_param('~min_range', 0.30)
        self.max_range = rospy.get_param('~max_range', 4.5)
        self.window = rospy.get_param('~accumulate', 1.5)   # seconds, 0 = off
        # Density filter over the accumulated window: keep a point only if
        # its 5 cm cell collected at least this many hits. Real obstacles
        # (even 1.5 cm tripod legs) rack up dozens of hits; single stray
        # noise returns would otherwise each cost a robot-half-width lethal
        # disk in the costmap for the whole accumulation window.
        self.min_hits = rospy.get_param('~min_hits', 3)
        self.cell = rospy.get_param('~density_cell', 0.05)
        # Skid-steer yaw slips badly while rotating in place, and Livox
        # frames captured mid-spin are internally smeared arcs. Above this
        # yaw rate (rad/s) new frames are NOT added to the accumulator
        # (the already-stored window stays valid - each frame is anchored
        # to its own odometry pose).
        self.max_spin = rospy.get_param('~max_spin_rate', 0.35)
        self.out_frame = rospy.get_param('~out_frame', 'base_link')
        odom_topic = rospy.get_param('~odom_topic', 'odometry/filtered')
        self.last_wz = 0.0

        c, s = np.cos(pitch), np.sin(pitch)
        # rows of R^T; p_base = p_sensor . rot_t + trans
        self.rot_t = np.array([[c, 0.0, -s],
                               [0.0, 1.0, 0.0],
                               [s, 0.0, c]])
        self.trans = np.array([mount_x, mount_y, mount_z])

        # odometry history: parallel lists (time, rotation, translation)
        self.odom_t = collections.deque(maxlen=300)   # ~6 s at 50 Hz
        self.odom_r = collections.deque(maxlen=300)
        self.odom_p = collections.deque(maxlen=300)

        self.buffer = collections.deque()  # (time_sec, points in odom frame)

        self.pub = rospy.Publisher('mid360/obstacles', PointCloud2, queue_size=2)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber('livox/lidar', PointCloud2, self.callback,
                         queue_size=2, buff_size=1 << 24)
        rospy.loginfo('mid360_filter: mount xyz=(%.3f, %.3f, %.3f) pitch=%.4f rad, '
                      'band z=[%.2f, %.2f] along world-up, accumulate %.1f s',
                      mount_x, mount_y, mount_z, pitch,
                      self.min_z, self.max_z, self.window)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_t.append(msg.header.stamp.to_sec())
        self.odom_r.append(quat_to_rot(q.x, q.y, q.z, q.w))
        self.odom_p.append(np.array([p.x, p.y, p.z]))
        self.last_wz = msg.twist.twist.angular.z

    def pose_at(self, t):
        """Odometry pose (R, p) closest in time to t, or None."""
        if not self.odom_t:
            return None
        times = list(self.odom_t)
        i = bisect.bisect_left(times, t)
        if i <= 0:
            i = 0
        elif i >= len(times):
            i = len(times) - 1
        elif t - times[i - 1] < times[i] - t:
            i -= 1
        return self.odom_r[i], self.odom_p[i]

    def callback(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return
        dtype = np.dtype({'names': [f.name for f in msg.fields],
                          'formats': [_FMT[f.datatype] for f in msg.fields],
                          'offsets': [f.offset for f in msg.fields],
                          'itemsize': msg.point_step})
        pts = np.frombuffer(msg.data, dtype=dtype, count=n)
        xyz = np.column_stack((pts['x'], pts['y'], pts['z'])).astype(np.float64)

        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0]:
            rng = np.sqrt((xyz ** 2).sum(axis=1))
            xyz = xyz[(rng > self.min_range) & (rng < self.max_range)]
        base = xyz.dot(self.rot_t) + self.trans if xyz.shape[0] else xyz

        stamp = msg.header.stamp.to_sec()
        pose = self.pose_at(stamp)

        # Height band along world-up. With odometry, use the gravity
        # direction from the EKF's roll/pitch (third row of the rotation);
        # without it, fall back to the robot z-axis.
        if base.shape[0]:
            up = pose[0][2, :] if pose is not None else np.array([0.0, 0.0, 1.0])
            h = base.dot(up)
            base = base[(h > self.min_z) & (h < self.max_z)]

        if pose is None or self.window <= 0.0:
            self.publish(msg, base)
            return

        # store in odom frame (pose matched to the cloud's own stamp),
        # output the whole window in the base frame at this stamp.
        # While spinning fast, the window FREEZES: mid-spin frames are
        # smeared (skid-steer slip + intra-frame motion) so they are not
        # stored, but the pre-spin window is kept alive - it stays valid
        # because every stored frame is anchored to its own odometry pose.
        rot, trans = pose
        if abs(self.last_wz) <= self.max_spin:
            self.buffer.append((stamp, base.dot(rot.T) + trans))
            cutoff = stamp - self.window
            while self.buffer and self.buffer[0][0] < cutoff:
                self.buffer.popleft()
        if not self.buffer:
            self.publish(msg, np.zeros((0, 3)))
            return
        merged_odom = np.vstack([b[1] for b in self.buffer])
        merged_base = (merged_odom - trans).dot(rot)   # == R^T . (p - t)

        # density filter: drop isolated points (sensor noise)
        if self.min_hits > 1 and merged_base.shape[0]:
            cells = np.floor(merged_base[:, :2] / self.cell).astype(np.int64)
            key = cells[:, 0] * 1000003 + cells[:, 1]
            _, inverse, counts = np.unique(key, return_inverse=True,
                                           return_counts=True)
            merged_base = merged_base[counts[inverse] >= self.min_hits]

        self.publish(msg, merged_base)

    def publish(self, msg, pts):
        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.out_frame
        out.height = 1
        out.width = pts.shape[0]
        out.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * out.width
        out.is_dense = True
        out.data = pts.astype(np.float32).tobytes()
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('mid360_filter')
    Mid360Filter()
    rospy.spin()
