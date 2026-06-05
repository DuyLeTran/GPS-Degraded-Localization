#!/usr/bin/env python3
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo

def main():
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri='data/UrbanNav_dataset/whampoa_ros2_bag',
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    
    found = False
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/zed2/camera/left/camera_info':
            msg = deserialize_message(data, CameraInfo)
            print("=== ZED2 Camera Info ===")
            print(f"Width: {msg.width}, Height: {msg.height}")
            print(f"K (Intrinsics): {msg.k}")
            print(f"P (Projection): {msg.p}")
            # K is a flat array of size 9: fx, 0, cx, 0, fy, cy, 0, 0, 1
            print(f"fx: {msg.k[0]}")
            print(f"fy: {msg.k[4]}")
            print(f"cx: {msg.k[2]}")
            print(f"cy: {msg.k[5]}")
            print("========================")
            found = True
            break
            
    if not found:
        print("Could not find /zed2/camera/left/camera_info topic in bag")

if __name__ == '__main__':
    main()
