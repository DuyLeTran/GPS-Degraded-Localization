#!/usr/bin/env python3
import sys
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

def measure(bag_path, topic='/gps/status'):
    typestore = get_typestore(Stores.ROS2_JAZZY)
    transitions = []
    prev_status, lost_time = None, None
    with Reader(bag_path) as reader:
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic == topic:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                status, t = msg.data, timestamp/1e9
                if prev_status and status != prev_status:
                    transitions.append({'time': t, 'from': prev_status, 'to': status})
                    if status == 'GPS_LOST': lost_time = t
                    elif status == 'GPS_GOOD' and lost_time:
                        print(f'Handover: {t-lost_time:.2f}s (LOST@{lost_time:.2f} → GOOD@{t:.2f})')
                        lost_time = None
                prev_status = status
    
    lats = []
    lt = None
    for tr in transitions:
        if tr['to'] == 'GPS_LOST': lt = tr['time']
        elif tr['to'] == 'GPS_GOOD' and lt: 
            lats.append(tr['time']-lt)
            lt = None
    
    if lats:
        mx = max(lats)
        print(f'\n=== KPI B6 ===\nMax Latency: {mx:.2f}s | Threshold: ≤ 2.0s | {"✅ PASS" if mx<=2.0 else "❌ FAIL"}')
    else:
        print('No LOST→GOOD transitions found.')

if __name__ == '__main__':
    measure(sys.argv[1] if len(sys.argv)>1 else 'data/ekf_result')
