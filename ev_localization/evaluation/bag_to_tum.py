#!/usr/bin/env python3
import sys
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

def bag_to_tum(bag_path, output_path, topic='/ekf/pose'):
    typestore = get_typestore(Stores.ROS2_JAZZY)
    with Reader(bag_path) as reader:
        with open(output_path, 'w') as f:
            for conn, timestamp, rawdata in reader.messages():
                if conn.topic == topic:
                    msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    p = msg.pose.position
                    q = msg.pose.orientation
                    f.write(f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                            f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
    print(f'Saved {output_path}')

if __name__ == '__main__':
    bag_to_tum(sys.argv[1] if len(sys.argv)>1 else 'data/ekf_result',
               sys.argv[2] if len(sys.argv)>2 else 'data/ekf_trajectory.tum')
