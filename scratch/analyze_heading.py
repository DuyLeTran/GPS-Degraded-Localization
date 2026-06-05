#!/usr/bin/env python3
import math

def main():
    gt_path = "/home/tranleduy/GPS-Degraded-Localization/data/UrbanNav_dataset/UrbanNav_whampoa_raw.txt"
    headings = []
    times = []
    
    with open(gt_path, 'r') as f:
        lines = f.readlines()
        
    for line in lines:
        line = line.strip()
        if not line:
            continue
        tokens = line.split()
        if tokens[0] == "UTCTime" or tokens[0] == "(sec)":
            continue
        try:
            t = float(tokens[0])
            h = float(tokens[18])
            times.append(t)
            headings.append(h)
        except Exception:
            continue
            
    if not headings:
        print("No heading data found!")
        return
        
    start_time = times[0]
    initial_heading = headings[0]
    print(f"Dataset starting UTCTime: {start_time}")
    print(f"Initial Heading: {initial_heading:.2f} deg")
    
    # Track heading change relative to start
    # Let's find U-turn (change close to 180 deg)
    uturn_found = False
    for i in range(1, len(headings)):
        elapsed = times[i] - start_time
        # Normalize diff to [-180, 180]
        diff = headings[i] - initial_heading
        diff = (diff + 180) % 360 - 180
        
        # Heading change of around 150-180 degrees
        if abs(diff) > 150.0 and not uturn_found:
            print(f"U-Turn detected at elapsed time: {elapsed:.2f}s (UTCTime: {times[i]:.2f}, Heading: {headings[i]:.2f} deg)")
            uturn_found = True
            
    # Also print final stats
    total_duration = times[-1] - start_time
    print(f"Total dataset duration: {total_duration:.2f} seconds")

if __name__ == '__main__':
    main()
