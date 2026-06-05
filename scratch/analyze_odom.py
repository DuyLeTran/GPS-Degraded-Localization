import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import math

def analyze():
    bag_path = 'data/urbannav_ekf_result'
    
    serialization_format = 'cdr'
    reader = rosbag2_py.SequentialReader()
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format
    )
    
    reader.open(storage_options, converter_options)
    
    topics = reader.get_all_topics_and_types()
    topic_types = {t.name: t.type for t in topics}
    
    odom_msgs = []
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == '/vehicle/odom':
            msg = deserialize_message(data, Odometry)
            # Use message stamp for time if possible
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom_msgs.append((t_sec, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            
    print(f"Total odom messages: {len(odom_msgs)}")
    if not odom_msgs:
        return
        
    omega_zs = [x[2] for x in odom_msgs]
    print(f"Min angular.z: {min(omega_zs)}, Max angular.z: {max(omega_zs)}")
    
    # Analyze heading changes with different window sizes
    for window_sz in [2.0, 5.0, 10.0, 15.0, 20.0]:
        max_delta = 0.0
        accumulated = 0.0
        history = []
        last_t = None
        
        for t_sec, v, omega in odom_msgs:
            if last_t is None:
                last_t = t_sec
                continue
            dt = t_sec - last_t
            last_t = t_sec
            
            accumulated += omega * dt
            history.append((t_sec, accumulated))
            
            while history and t_sec - history[0][0] > window_sz:
                history.pop(0)
                
            if len(history) >= 2:
                delta = abs(history[-1][1] - history[0][1])
                if delta > max_delta:
                    max_delta = delta
                    
        print(f"Window size: {window_sz}s -> Max delta heading: {math.degrees(max_delta):.2f} degrees")

if __name__ == '__main__':
    analyze()
