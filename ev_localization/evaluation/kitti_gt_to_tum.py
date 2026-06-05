#!/usr/bin/env python3
import os, sys, math
import numpy as np

def load_oxts(oxts_dir, timestamps_file):
    poses = []
    import datetime
    with open(timestamps_file, 'r') as f:
        timestamps = []
        for line in f:
            if not line.strip():
                continue
            dt = datetime.datetime.strptime(line.strip()[:-3], '%Y-%m-%d %H:%M:%S.%f')
            timestamps.append(float(dt.strftime('%s.%f')))
    
    oxts_files = sorted([f for f in os.listdir(oxts_dir) if f.endswith('.txt')])
    first = np.loadtxt(os.path.join(oxts_dir, oxts_files[0]))
    ref_lat, ref_lon = math.radians(first[0]), math.radians(first[1])
    R = 6378137.0
    
    for i, fname in enumerate(oxts_files):
        d = np.loadtxt(os.path.join(oxts_dir, fname))
        dlat = math.radians(d[0]) - ref_lat
        dlon = math.radians(d[1]) - ref_lon
        x = R * dlon * math.cos(ref_lat)
        y = R * dlat
        yaw = d[5]
        qz, qw = math.sin(yaw/2), math.cos(yaw/2)
        poses.append((timestamps[i], x, y, d[2], 0, 0, qz, qw))
    return poses

if __name__ == '__main__':
    od = sys.argv[1] if len(sys.argv)>1 else 'data/kitti_dataset/2011_09_26/2011_09_26_drive_0009_sync/oxts/data'
    tf = sys.argv[2] if len(sys.argv)>2 else 'data/kitti_dataset/2011_09_26/2011_09_26_drive_0009_sync/oxts/timestamps.txt'
    out = sys.argv[3] if len(sys.argv)>3 else 'data/ground_truth.tum'
    poses = load_oxts(od, tf)
    with open(out, 'w') as f:
        for p in poses:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f} {p[4]:.6f} {p[5]:.6f} {p[6]:.6f} {p[7]:.6f}\n')
    print(f'Saved {out} with {len(poses)} poses')
