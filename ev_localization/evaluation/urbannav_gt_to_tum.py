#!/usr/bin/env python3
import sys
import math
import numpy as np

def dms_to_decimal(deg_str, min_str, sec_str):
    deg = float(deg_str)
    minute = float(min_str)
    sec = float(sec_str)
    decimal = abs(deg) + minute / 60.0 + sec / 3600.0
    if deg < 0 or deg_str.strip().startswith('-'):
        decimal = -decimal
    return decimal

def gps_to_local(lat, lon, ref_lat, ref_lon):
    R = 6378137.0
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    ref_lat_rad, ref_lon_rad = math.radians(ref_lat), math.radians(ref_lon)
    dlat, dlon = lat_rad - ref_lat_rad, lon_rad - ref_lon_rad
    x = R * dlon * math.cos(ref_lat_rad)
    y = R * dlat
    return x, y

def convert_gt(gt_path, output_path, ref_lat=22.30172976, ref_lon=114.18836329, initial_yaw=-2.434313):
    with open(gt_path, 'r') as f:
        lines = f.readlines()
        
    poses = []
    c_yaw = math.cos(-initial_yaw)
    s_yaw = math.sin(-initial_yaw)
    gps_rot = np.array([[c_yaw, -s_yaw],
                        [s_yaw,  c_yaw]])
                        
    for line in lines:
        line = line.strip()
        if not line:
            continue
        tokens = line.split()
        if tokens[0] == "UTCTime" or tokens[0] == "(sec)":
            continue
        try:
            utc_time = float(tokens[0])
            lat = dms_to_decimal(tokens[3], tokens[4], tokens[5])
            lon = dms_to_decimal(tokens[6], tokens[7], tokens[8])
            alt = float(tokens[9])
            heading_deg = float(tokens[18])
            
            # Convert to local ENU
            x_enu, y_enu = gps_to_local(lat, lon, ref_lat, ref_lon)
            
            # Rotate to body/local frame
            local_xy = gps_rot @ np.array([x_enu, y_enu])
            x, y = local_xy[0], local_xy[1]
            
            # Heading/yaw in body frame
            # Heading in text is degrees, convert to radians
            heading_rad = math.radians(heading_deg)
            # Subtract initial_yaw to align with odom frame heading
            local_yaw = heading_rad - initial_yaw
            
            qz = math.sin(local_yaw / 2.0)
            qw = math.cos(local_yaw / 2.0)
            
            poses.append((utc_time, x, y, alt, 0.0, 0.0, qz, qw))
        except Exception as e:
            continue
            
    with open(output_path, 'w') as f:
        for p in poses:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f} '
                    f'{p[4]:.6f} {p[5]:.6f} {p[6]:.6f} {p[7]:.6f}\n')
                    
    print(f'Saved {len(poses)} ground truth poses to {output_path}')

if __name__ == '__main__':
    gt_in = sys.argv[1] if len(sys.argv) > 1 else 'data/UrbanNav_dataset/UrbanNav_whampoa_raw.txt'
    tum_out = sys.argv[2] if len(sys.argv) > 2 else 'data/ground_truth.tum'
    convert_gt(gt_in, tum_out)
