#!/usr/bin/env python
"""Mid-360 obstacle filter (TF-free) with ego-motion-compensated accumulation.

Subscribes to the raw Livox cloud (/livox/lidar, PointCloud2, sensor frame),
applies the FIXED mount transform (parameters, not TF - another system on
this robot fights over the livox_frame TF), keeps only points inside the
robot's obstacle height band, accumulates the last ~1.5 s of points
(compensated with wheel/EKF odometry so the robot's own motion does not
smear them), and republishes everything in base_link frame on
~mid360/obstacles.

Why accumulate: thin obstacles (tripod legs, stool poles, ~1-2 cm tubes)
only get a few lidar hits per frame, and costmap raytrace-clearing erases a
cell whenever a ray passes BESIDE the thin object through the same cell.
Single-frame marking therefore flickers and the planner can drive through a
momentarily-cleared leg. With accumulation every costmap update re-marks
the leg from many hits, so marks are stable; it also keeps obstacles marked
briefly after they leave the tilted FOV during a close approach.

Heights are relative to base_link (~0.065 m above the floor):
  min_z 0.05 -> keep from ~0.12 m above the floor (low enough to see the
                star-base of a stool, high enough to reject the floor now
                that the mount pitch is measured, not guessed)
  max_z 0.50 -> ignore above ~0.57 m (robot is ~0.50 m tall; the 0.75 m
                bridge deck stays above the band)
"""
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
        self.min_z = rospy.get_param('~min_z', 0.05)     # base_link frame
        self.max_z = rospy.get_param('~max_z', 0.50)
        self.min_range = rospy.get_param('~min_range', 0.30)
        self.max_range = rospy.get_param('~max_range', 4.5)
        self.window = rospy.get_param('~accumulate', 1.5)   # seconds, 0 = off
        self.out_frame = rospy.get_param('~out_frame', 'base_link')
        odom_topic = rospy.get_param('~odom_topic', 'odometry/filtered')

        c, s = np.cos(pitch), np.sin(pitch)
        # rows of R^T; p_base = p_sensor . rot_t + trans
        self.rot_t = np.array([[c, 0.0, -s],
                               [0.0, 1.0, 0.0],
                               [s, 0.0, c]])
        self.trans = np.array([mount_x, mount_y, mount_z])

        self.odom_pose = None            # (R, t): base_link pose in odom
        self.buffer = collections.deque()  # (time_sec, points in odom frame)

        self.pub = rospy.Publisher('mid360/obstacles', PointCloud2, queue_size=2)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber('livox/lidar', PointCloud2, self.callback,
                         queue_size=2, buff_size=1 << 24)
        rospy.loginfo('mid360_filter: mount xyz=(%.3f, %.3f, %.3f) pitch=%.4f rad, '
                      'band z=[%.2f, %.2f] rel. base_link, accumulate %.1f s',
                      mount_x, mount_y, mount_z, pitch,
                      self.min_z, self.max_z, self.window)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_pose = (quat_to_rot(q.x, q.y, q.z, q.w),
                          np.array([p.x, p.y, p.z]))

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
        if base.shape[0]:
            base = base[(base[:, 2] > self.min_z) & (base[:, 2] < self.max_z)]

        if self.odom_pose is None or self.window <= 0.0:
            self.publish(msg, base)
            return

        # store in odom frame, output the whole window in the CURRENT base frame
        rot, trans = self.odom_pose
        now = msg.header.stamp.to_sec()
        self.buffer.append((now, base.dot(rot.T) + trans))
        cutoff = now - self.window
        while self.buffer and self.buffer[0][0] < cutoff:
            self.buffer.popleft()
        merged_odom = np.vstack([b[1] for b in self.buffer])
        merged_base = (merged_odom - trans).dot(rot)   # == R^T . (p - t)
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
