# Write the icp_odometry settings to try, one YAML file each, into /tmp/cfg_<name>.yaml
BASE = {
    'frame_id': 'base_link', 'odom_frame_id': 'odom', 'wait_for_transform': 0.2,
    'publish_tf': True, 'approx_sync': False,
    'Reg/Strategy': '1', 'Reg/Force3DoF': 'true', 'Icp/PointToPlane': 'false',
    'Icp/VoxelSize': '0.05', 'Icp/MaxCorrespondenceDistance': '0.3', 'Icp/Iterations': '15',
    'Icp/CorrespondenceRatio': '0.2', 'Icp/MaxTranslation': '1.0', 'Optimizer/GravitySigma': '0',
    'OdomF2M/ScanSubtractRadius': '0.05', 'OdomF2M/ScanMaxSize': '2000',
    'Odom/ResetCountdown': '1',
}
P2P = {'Icp/PointToPlane': 'true', 'Icp/PointToPlaneK': '10', 'Icp/PointToPlaneRadius': '0',
       'Icp/MaxCorrespondenceDistance': '0.15', 'Icp/Iterations': '30', 'Icp/Epsilon': '0.001'}
BIGMAP = {'OdomF2M/ScanMaxSize': '10000', 'Odom/ScanKeyFrameThr': '0.7'}
CONFIGS = {
    'now': {},
    'p2p': P2P,
    'p2p_big': {**P2P, **BIGMAP},
    'fine': {'Icp/MaxCorrespondenceDistance': '0.1', 'Icp/Iterations': '30', **BIGMAP},
    'p2p_big_far': {**P2P, **BIGMAP, 'Icp/RangeMax': '0', 'Icp/VoxelSize': '0.03'},
    'p2p_big_k5': {**P2P, **BIGMAP, 'Icp/PointToPlaneK': '5'},
    'deskew': {'deskewing': True},
    'deskew_slerp': {'deskewing': True, 'deskewing_slerp': True},
    # 16 Nov, for the cloud fixing chat: one change each from today's settings, so each run
    # answers one question (collect_drift_data.sh runs them).
    'big20k': {'OdomF2M/ScanMaxSize': '20000'},     # does it forget the room too fast?
    'mcd06': {'Icp/MaxCorrespondenceDistance': '0.6', 'Icp/Iterations': '30'},  # turns too fast?
    'p2l': {'Icp/PointToPlane': 'true'},            # match points to wall lines
    'vox0': {'Icp/VoxelSize': '0'},                 # keep every point (close things count more)
    'f2f': {'Odom/Strategy': '1'},                  # each sweep against the last one only
    'noguess': {'Odom/GuessMotion': 'false'},       # no "keeps moving the same way" guess
    # 16 Nov evening (campus PC chat): the tracker's memory. It adds a sweep to its memory when
    # the match score is under Odom/ScanKeyFrameThr (0.9). The score counts against all 720 beams
    # and a third never return, so it tops out near 0.55: every sweep is added, the 2000 points
    # turn over in about a second, and each sweep is matched against a smear of the last few.
    'kf04': {'Odom/ScanKeyFrameThr': '0.4'},        # add a sweep only when the view changed
    'kf035': {'Odom/ScanKeyFrameThr': '0.35'},
    'kf04_big': {'Odom/ScanKeyFrameThr': '0.4', 'OdomF2M/ScanMaxSize': '20000'},
    'kf035_big': {'Odom/ScanKeyFrameThr': '0.35', 'OdomF2M/ScanMaxSize': '20000'},
    'ssa0': {'OdomF2M/ScanSubtractAngle': '0'},     # a point near a remembered one is never added
    'ssa0_big': {'OdomF2M/ScanSubtractAngle': '0', 'OdomF2M/ScanMaxSize': '20000'},
    'nonorm': {'Icp/PointToPlaneK': '0'},           # no surface directions at all
    'range30': {'OdomF2M/ScanRange': '30', 'OdomF2M/ScanMaxSize': '20000'},   # memory by place
    'kf045_big': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '20000'},
    'kf05_big': {'Odom/ScanKeyFrameThr': '0.5', 'OdomF2M/ScanMaxSize': '20000'},
    'kf045_big_r2': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '20000'},  # repeat
    'kf043_big': {'Odom/ScanKeyFrameThr': '0.43', 'OdomF2M/ScanMaxSize': '20000'},
    'kf047_big': {'Odom/ScanKeyFrameThr': '0.47', 'OdomF2M/ScanMaxSize': '20000'},
    'kf045_10k': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '10000'},
    'kf045_5k': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '5000'},
    'kf045': {'Odom/ScanKeyFrameThr': '0.45'},
    # the settings in stage3.launch.py (16 Nov late: memory 10000, as accurate as 20000 and
    # half the time per sweep). fix_20k is the first choice, run as 'fix' on 16 Nov evening.
    'fix': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '10000'},
    # 16 Nov night: the pusher (or handle) at 0.16 to 0.3 m left-rear rides along in every turn;
    # the tracker used every point down to 0 m. Ignore close points in the tracker only.
    'fix_rmin04': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '10000',
                   'scan_range_min': 0.4},
    'fix_rmin06': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '10000',
                   'scan_range_min': 0.6},
    # 16 Nov night: correct each sweep for the device turning while the LiDAR spins.
    'fix_deskew': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '10000',
                   'deskewing': True},
    'fix_deskew_slerp': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '10000',
                         'deskewing': True, 'deskewing_slerp': True},
    'fix_20k': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '20000'},
    'fix_r2': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '20000'},
    'fix_10k_r2': {'Odom/ScanKeyFrameThr': '0.45', 'OdomF2M/ScanMaxSize': '10000'},
    'kf04_big_ssa0': {'Odom/ScanKeyFrameThr': '0.4', 'OdomF2M/ScanMaxSize': '20000',
                      'OdomF2M/ScanSubtractAngle': '0'},
}
for name, extra in CONFIGS.items():
    params = {**BASE, **extra}
    with open('/tmp/cfg_%s.yaml' % name, 'w') as f:
        f.write('/**:\n  ros__parameters:\n')
        for k, v in params.items():
            if isinstance(v, bool):
                f.write('    %s: %s\n' % (k, 'true' if v else 'false'))
            elif isinstance(v, float):
                f.write('    %s: %s\n' % (k, v))
            else:
                f.write("    %s: '%s'\n" % (k, v))
print('written:', ', '.join(CONFIGS))
