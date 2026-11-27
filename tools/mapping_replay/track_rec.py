# During a replay: keep every movement-tracker pose (/odom), its own account of each sweep
# (/odom_info, since 16 Nov) and every 5th filtered sweep (/scan),
# so plot_track.py can draw the path with what the LiDAR saw around it.
#   python3 track_rec.py OUT.npz        (stops by itself 5 s after /odom goes quiet)
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
try:
    from rtabmap_msgs.msg import OdomInfo
except ImportError:
    OdomInfo = None

rclpy.init()
n = Node('track_rec')
poses, sweeps, info = [], [], []
state = {'at': 0.0, 'scan_count': 0, 'meta': None}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def on_odom(m):
    state['at'] = time.time()
    q = m.pose.pose.orientation
    if q.w == 0 and q.x == 0 and q.y == 0 and q.z == 0:
        return                                  # lost
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    poses.append((stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw))


def on_scan(m):
    state['scan_count'] += 1
    if state['meta'] is None:
        state['meta'] = (stamp(m.header), m.angle_min, m.angle_increment, len(m.ranges))
    if state['scan_count'] % 5 == 0 and len(m.ranges) == state['meta'][3]:
        sweeps.append((stamp(m.header), np.array(m.ranges, dtype=np.float32)))




def on_info(m):
    # the tracker's own account of each sweep: match ratio, memory size, was the sweep added to
    # the memory, time taken, matched points, lost
    info.append((stamp(m.header), m.icp_inliers_ratio, m.local_scan_map_size, float(m.key_frame_added),
                 m.time_estimation, m.icp_correspondences, float(m.lost)))


n.create_subscription(Odometry, '/odom', on_odom, 100)
if OdomInfo is not None:
    n.create_subscription(OdomInfo, '/odom_info', on_info, 100)
n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
start = time.time()
while time.time() - start < 700:
    rclpy.spin_once(n, timeout_sec=0.2)
    if poses and time.time() - state['at'] > 5:
        break
meta = state['meta'] or (0.0, 0.0, 0.0, 0)
np.savez_compressed(sys.argv[1], poses=np.array(poses), sweep_t=np.array([t for t, _ in sweeps]),
                    sweep_r=np.array([r for _, r in sweeps]), meta=np.array(meta),
                    info=np.array(info))
print('saved %d poses, %d sweeps to %s' % (len(poses), len(sweeps), sys.argv[1]))
