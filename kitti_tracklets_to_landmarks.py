#!/usr/bin/env python3
import json, sys
import xml.etree.ElementTree as ET

def parse_tracklets(tracklet_file):
    tree = ET.parse(tracklet_file)
    root = tree.getroot()
    tracklets = root.find('tracklets')
    landmarks = []
    lm_id = 1
    for item in tracklets.findall('item'):
        obj_type = item.find('objectType').text
        h = float(item.find('h').text)
        w = float(item.find('w').text)
        poses = item.find('poses')
        first_pose = poses.findall('item')[0]
        tx = float(first_pose.find('tx').text)
        ty = float(first_pose.find('ty').text)
        tz = float(first_pose.find('tz').text)
        landmarks.append({
            "id": lm_id, "class": obj_type.lower(),
            "position_enu": [tx, ty, tz],
            "descriptor": [0.0]*4,
            "t_first": 0.0, "t_last": 0.0, "n_obs": 1,
            "bbox_size": [w, h]
        })
        lm_id += 1
    return {"landmarks": landmarks}

if __name__ == '__main__':
    tf = sys.argv[1] if len(sys.argv)>1 else 'data/kitti_dataset/2011_09_26/2011_09_26_drive_0046_sync/tracklet_labels.xml'
    out = 'ev_localization/config/landmarks_kitti.json'
    result = parse_tracklets(tf)
    with open(out, 'w') as f: json.dump(result, f, indent=4)
    print(f'Created {out} with {len(result["landmarks"])} landmarks')
