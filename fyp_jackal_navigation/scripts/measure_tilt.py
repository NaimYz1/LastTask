#!/usr/bin/env python
"""Measure the Mid-360 mount tilt from the raw cloud - no TF involved.

Settles the 38.22-vs-45-degree question empirically: with the robot
STATIONARY on flat open floor (2-4 m of clear space ahead, no boxes/feet in
front), the floor plane is fitted in the raw sensor frame and the mount
angles fall out of the plane normal.

Usage (Livox driver must be running):
    rosrun fyp_jackal_navigation measure_tilt.py

Reports nose-down pitch, lateral roll, and the sensor's height above the
floor. The height is the built-in sanity check: it should match the tape
measure (~0.455 m on this robot). If the height is way off, the dominant
plane it found was not the floor - clear the area and rerun.
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField

_FMT = {PointField.INT8: np.int8, PointField.UINT8: np.uint8,
        PointField.INT16: np.int16, PointField.UINT16: np.uint16,
        PointField.INT32: np.int32, PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}


def cloud_to_xyz(msg):
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    dtype = np.dtype({'names': [f.name for f in msg.fields],
                      'formats': [_FMT[f.datatype] for f in msg.fields],
                      'offsets': [f.offset for f in msg.fields],
                      'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dtype, count=n)
    xyz = np.column_stack((pts['x'], pts['y'], pts['z'])).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


class TiltMeasure(object):
    def __init__(self):
        self.want_frames = rospy.get_param('~frames', 20)
        # gate for candidate floor planes: within 30 deg of this pitch guess,
        # wide enough to accept both 38.22 and 45.
        self.guess = np.radians(rospy.get_param('~expected_pitch_deg', 41.5))
        self.frames = []
        self.done = False
        self.sub = rospy.Subscriber('livox/lidar', PointCloud2, self.callback,
                                    queue_size=5, buff_size=1 << 24)

    def callback(self, msg):
        if self.done:
            return
        self.frames.append(cloud_to_xyz(msg))
        rospy.loginfo('collected frame %d/%d', len(self.frames), self.want_frames)
        if len(self.frames) >= self.want_frames:
            self.done = True
            self.sub.unregister()
            self.solve()
            rospy.signal_shutdown('done')

    def solve(self):
        pts = np.vstack(self.frames)
        rng = np.sqrt((pts ** 2).sum(axis=1))
        pts = pts[(rng > 0.5) & (rng < 4.0)]
        if pts.shape[0] < 2000:
            rospy.logerr('Too few points (%d) between 0.5 and 4 m - is there '
                         'open floor ahead?', pts.shape[0])
            return
        if pts.shape[0] > 40000:
            pts = pts[np.random.RandomState(1).choice(pts.shape[0], 40000,
                                                      replace=False)]

        ref = np.array([-np.sin(self.guess), 0.0, np.cos(self.guess)])
        min_dot = np.cos(np.radians(30.0))
        rs = np.random.RandomState(42)
        best_count, best_inliers = 0, None
        for _ in range(400):
            p = pts[rs.choice(pts.shape[0], 3, replace=False)]
            n = np.cross(p[1] - p[0], p[2] - p[0])
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n /= norm
            if n[2] < 0:
                n = -n
            if n.dot(ref) < min_dot:   # not floor-like (e.g. a wall)
                continue
            dist = np.abs(pts.dot(n) - p[0].dot(n))
            inliers = dist < 0.02
            count = inliers.sum()
            if count > best_count:
                best_count, best_inliers = count, inliers
        if best_inliers is None or best_count < 1000:
            rospy.logerr('No floor-like plane found (best had %d inliers). '
                         'Is the robot on open flat ground?', best_count)
            return

        # least-squares refinement on the inliers
        floor = pts[best_inliers]
        centroid = floor.mean(axis=0)
        _, _, vt = np.linalg.svd(floor - centroid, full_matrices=False)
        n = vt[2]
        if n[2] < 0:
            n = -n

        pitch = np.arctan2(-n[0], n[2])
        roll = np.arctan2(n[1], n[2])
        height = abs(centroid.dot(n))

        print('')
        print('==================== MID-360 MOUNT MEASUREMENT ====================')
        print('Floor plane: %d inlier points (of %d used)' % (best_count, pts.shape[0]))
        print('Nose-down pitch : %7.2f deg   (%.4f rad)' % (np.degrees(pitch), pitch))
        print('Lateral roll    : %7.2f deg   (should be near 0)' % np.degrees(roll))
        print('Sensor height   : %7.3f m    (tape measure said ~0.455;' % height)
        print('                                if this is far off, rerun on clearer floor)')
        print('')
        print('If the pitch differs from the current default (45.00 deg), launch with:')
        print('  roslaunch fyp_jackal_navigation nav_real.launch '
              'mid360_pitch:=%.4f mid360_z:=%.3f' % (pitch, height - 0.065))
        print('====================================================================')


if __name__ == '__main__':
    rospy.init_node('measure_tilt')
    TiltMeasure()
    rospy.loginfo('Waiting for /livox/lidar... keep the robot STATIONARY on '
                  'flat floor with 2-4 m clear ahead.')
    rospy.spin()
