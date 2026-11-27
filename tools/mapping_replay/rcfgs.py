# rtabmap settings to try: today's (loop closing effectively off) and with it on.
BASE = {
    'frame_id': 'base_link', 'odom_frame_id': 'odom', 'map_frame_id': 'rtabmap_map',
    'approx_sync': False, 'wait_for_transform': 0.2, 'subscribe_scan': True,
    'subscribe_depth': False, 'subscribe_rgb': False, 'subscribe_rgbd': False,
    'Rtabmap/DetectionRate': '1.0', 'RGBD/LinearUpdate': '0.05', 'RGBD/AngularUpdate': '0.05',
    'Grid/Sensor': '0', 'Grid/CellSize': '0.05', 'Grid/RangeMin': '0.3', 'Grid/RangeMax': '8.0',
    'Grid/RayTracing': 'true', 'RGBD/ProximityBySpace': 'true',
    'RGBD/NeighborLinkRefining': 'true', 'Mem/NotLinkedNodesKept': 'false',
    'Reg/Strategy': '1', 'Reg/Force3DoF': 'true', 'Icp/PointToPlane': 'false',
    'Icp/VoxelSize': '0.05', 'Icp/MaxCorrespondenceDistance': '0.3', 'Icp/Iterations': '15',
    'Icp/MaxTranslation': '1.0', 'Optimizer/GravitySigma': '0',
    'Icp/CorrespondenceRatio': '0.5',
}
LOOP = {'RGBD/ProximityMaxGraphDepth': '0', 'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/LocalRadius': '5', 'Icp/MaxTranslation': '3.0', 'RGBD/OptimizeMaxError': '3.0'}
CONFIGS = {
    'r_now': {},
    'r_045': {'Icp/CorrespondenceRatio': '0.45'},
    'r_035': {'Icp/CorrespondenceRatio': '0.35'},
    'r_035_loop': {'Icp/CorrespondenceRatio': '0.35', **LOOP},
    'r_025_loop': {'Icp/CorrespondenceRatio': '0.25', **LOOP},
    'r_02_loop': {'Icp/CorrespondenceRatio': '0.2', **LOOP},
    'r_015_loop': {'Icp/CorrespondenceRatio': '0.15', **LOOP},
    # 15 Nov (2): a wider match distance, so the end can snap to START from ~1.8 m off. Worse.
    'r_02_mcd1': {'Icp/CorrespondenceRatio': '0.2', **LOOP,
                  'Icp/MaxCorrespondenceDistance': '1.0', 'Icp/Iterations': '30'},
    # 15 Nov (2): START is only tried if the map's guess is within this radius (default 1 m).
    'r_02_pfr3': {'Icp/CorrespondenceRatio': '0.2', **LOOP,
                  'RGBD/ProximityPathFilteringRadius': '3'},
    'r_02_pfr3_mcd06': {'Icp/CorrespondenceRatio': '0.2', **LOOP,
                        'RGBD/ProximityPathFilteringRadius': '3',
                        'Icp/MaxCorrespondenceDistance': '0.6',
                        'RGBD/NeighborLinkRefining': 'false'},
}
for name, extra in CONFIGS.items():
    with open('/tmp/rcfg_%s.yaml' % name, 'w') as f:
        f.write('/**:\n  ros__parameters:\n')
        for k, v in {**BASE, **extra}.items():
            if isinstance(v, bool):
                f.write('    %s: %s\n' % (k, 'true' if v else 'false'))
            elif isinstance(v, float):
                f.write('    %s: %s\n' % (k, v))
            else:
                f.write("    %s: '%s'\n" % (k, v))
print('written:', ', '.join(CONFIGS))
