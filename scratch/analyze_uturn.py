#!/usr/bin/env python3
import math

def main():
    gt_path = "/home/tranleduy/GPS-Degraded-Localization/data/UrbanNav_dataset/UrbanNav_whampoa_raw.txt"
    with open(gt_path, 'r') as f:
        lines = f.readlines()
        
    print("Time(s) | Lat | Lon | Heading(deg) | VelForward(m/s)")
    print("-" * 60)
    
    start_time = None
    for line in lines:
        line = line.strip()
        if not line:
            continue
        tokens = line.split()
        if tokens[0] == "UTCTime" or tokens[0] == "(sec)":
            continue
        try:
            t = float(tokens[0])
            if start_time is None:
                start_time = t
            elapsed = t - start_time
            
            # Print every 5 seconds in the range of 530s to 600s
            if 530.0 <= elapsed <= 600.0:
                if int(elapsed) % 5 == 0:
                    h = float(tokens[18])
                    vel = float(tokens[11])
                    print(f"{elapsed:7.1f} | {tokens[3]} {tokens[4]} {tokens[5]} | {tokens[6]} {tokens[7]} {tokens[8]} | {h:12.2f} | {vel:14.2f}")
        except Exception as e:
            continue

if __name__ == '__main__':
    main()
