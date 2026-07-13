#!/usr/bin/env python
"""Mid-360 obstacle filter (TF-free).

Subscribes to the raw Livox cloud (/livox/lidar, PointCloud2, sensor frame),
applies the FIXED mount transform (parameters, not TF), keeps only points
inside the robot's obstacle height band, and republishes in base_link frame
(~mid360/obstacles).

Why not TF: on this robot another system also publishes a pose for
livox_frame (lidar odometry), so the frame's parent flip-flops and the
costmap projected the cloud through garbage transforms (sensor origin at
z=-0.14, phantom obstacles everywhere). Doing the fixed-mount math here and
stamping the output with base_link makes navigation immune to that, and to
odometry z-drift as well (the height band is applied relative to the robot,
where it belongs).

Heights are relative to base_link, which sits ~0.065 m above the floor:
  min_z 0.12 -> ignore anything below ~0.19 m above the floor (ground noise)
  max_z 0.50 -> ignore anything above ~0.57 m above the floor (robot is
                ~0.50 m tall; the 0.75 m bridge deck stays above this band)
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField

# PointField datatype -> numpy dtype
_FMT = {PointField.INT8: np.int8, PointField.UINT8: np.uint8,
        PointField.INT16: np.int16, PointField.UINT16: np.uint16,
        PointField.INT32: np.int32, PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}


class Mid360Filter(object):
    def __init__(self):
        mount_x = rospy.get_param('~mount_x', 0.179)
        mount_y = rospy.get_param('~mount_y', 0.0)
        mount_z = rospy.get_param('~mount_z', 0.394)
        pitch = rospy.get_param('~mount_pitch', 0.7854)  # rad, nose-down
        self.min_z = rospy.get_param('~min_z', 0.12)     # base_link frame
        self.max_z = rospy.get_param('~max_z', 0.50)
        self.min_range = rospy.get_param('~min_range', 0.30)
        self.max_range = rospy.get_param('~max_range', 4.5)
        self.out_frame = rospy.get_param('~out_frame', 'base_link')

        c, s = np.cos(pitch), np.sin(pitch)
        # rows of R^T; p_base = p_sensor . rot_t + trans
        self.rot_t = np.array([[c, 0.0, -s],
                               [0.0, 1.0, 0.0],
                               [s, 0.0, c]])
        self.trans = np.array([mount_x, mount_y, mount_z])

        self.pub = rospy.Publisher('mid360/obstacles', PointCloud2, queue_size=2)
        rospy.Subscriber('livox/lidar', PointCloud2, self.callback,
                         queue_size=2, buff_size=1 << 24)
        rospy.loginfo('mid360_filter: mount xyz=(%.3f, %.3f, %.3f) pitch=%.4f rad, '
                      'band z=[%.2f, %.2f] rel. base_link',
                      mount_x, mount_y, mount_z, pitch, self.min_z, self.max_z)

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

        good = np.isfinite(xyz).all(axis=1)
        xyz = xyz[good]
        if xyz.shape[0] == 0:
            self.publish(msg, xyz)
            return
        rng = np.sqrt((xyz ** 2).sum(axis=1))
        xyz = xyz[(rng > self.min_range) & (rng < self.max_range)]

        base = xyz.dot(self.rot_t) + self.trans
        base = base[(base[:, 2] > self.min_z) & (base[:, 2] < self.max_z)]
        self.publish(msg, base)

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
