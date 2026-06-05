#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import sys

class UTurnLiveTester(Node):
    def __init__(self):
        super().__init__('uturn_live_tester', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, False)
        ])
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odom', 10)
        self.event_sub = self.create_subscription(String, '/vehicle/u_turn_event', self.event_callback, 10)
        
        self.event_received = False
        self.event_time = None
        
    def event_callback(self, msg):
        if msg.data == 'U_TURN_DETECTED':
            self.event_received = True
            self.get_logger().info('Received U_TURN_DETECTED event!')

def main():
    rclpy.init()
    tester = UTurnLiveTester()
    
    print("Waiting for uturn_detector node to start up...")
    time.sleep(2)
    
    print("Starting simulation of U-turn (yaw rate = 1.5 rad/s)...")
    start_wall = time.time()
    detected_at_wall = None
    
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.twist.twist.angular.z = 1.5  # 1.5 rad/s
    
    # Publish at ~20Hz (every 0.05s) for 2.5 seconds
    for _ in range(50):
        if not rclpy.ok():
            break
            
        # Update timestamp
        now = tester.get_clock().now()
        msg.header.stamp = now.to_msg()
        
        tester.odom_pub.publish(msg)
        rclpy.spin_once(tester, timeout_sec=0.005)
        
        if tester.event_received and detected_at_wall is None:
            detected_at_wall = time.time()
            
        time.sleep(0.05)
        
    # Check results
    if tester.event_received:
        elapsed_to_detection = detected_at_wall - start_wall
        # Threshold 150 deg (2.618 rad) is crossed at t = 2.618 / 1.5 = 1.745 seconds.
        latency = elapsed_to_detection - 1.745
        print(f"SUCCESS: U-turn event detected after {elapsed_to_detection:.3f} seconds!")
        print(f"Estimated detection latency: {latency:.3f} seconds.")
        if latency <= 1.0:
            print("LATENCY REQUIREMENT MET (<= 1.0s)!")
            sys.exit(0)
        else:
            print("FAILURE: Latency exceeded 1.0s!")
            sys.exit(1)
    else:
        print("FAILURE: U-turn event was not detected within the test window.")
        sys.exit(1)

if __name__ == '__main__':
    main()
