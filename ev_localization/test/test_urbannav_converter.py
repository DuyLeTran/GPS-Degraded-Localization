#!/usr/bin/env python3
import pytest
import os
import tempfile
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

from ev_localization.urbannav_converter import UrbanNavConverter

# Mock data content similar to UrbanNav_whampoa_raw.txt
MOCK_GT_CONTENT = """      UTCTime       Week   GPSTime         Latitude        Longitude        H-Ell VelBdyX VelBdyY VelBdyZ AccBdyX AccBdyY AccBdyZ           Roll          Pitch        Heading Q
         (sec)    (weeks)     (sec)       (+/-D M S)       (+/-D M S)          (m)   (m/s)   (m/s)   (m/s) (m/s^2) (m/s^2) (m/s^2)          (deg)          (deg)          (deg)  
 1621578524.00 2158.00000 455342.00   22 18 05.61075  114 11 25.11071        2.894  -0.004   1.500  -0.007   0.209   0.093  -0.092  -0.2074966655  -0.6303452479 -59.3645242579 1
 1621578525.00 2158.00000 455343.00   22 18 05.61065  114 11 25.11062        2.884  -0.006   2.000  -0.010  -0.267  -0.160   0.164  -0.2035719046  -0.6310327972 -59.3649654354 1
 1621578526.00 2158.00000 455344.00   22 18 05.61055  114 11 25.11053        2.874  -0.004   2.500  -0.009  -0.065  -0.100  -0.045  -0.2023686932  -0.6317432649 -59.3656816326 1
 1621578527.00 2158.00000 455345.00   22 18 05.61044  114 11 25.11042        2.863  -0.006   3.000  -0.009   0.423   0.167  -0.066  -0.2022827496  -0.6350664201 -59.3632064549 1
"""

@pytest.fixture(scope="module")
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()

def test_dms_to_decimal(ros_init):
    # Create a temporary file to initialize converter
    with tempfile.NamedTemporaryFile(mode='w+', delete=False) as f:
        f.write(MOCK_GT_CONTENT)
        temp_path = f.name

    try:
        # Instantiate converter with mock path
        node = UrbanNavConverter(parameter_overrides=[
            Parameter('gt_path', Parameter.Type.STRING, temp_path)
        ])
        
        # Test basic conversions
        lat = node.dms_to_decimal("22", "18", "05.61075")
        lon = node.dms_to_decimal("114", "11", "25.11071")
        
        # 22 + 18/60 + 05.61075/3600 = 22.30155854
        assert abs(lat - 22.30155854) < 1e-6
        # 114 + 11/60 + 25.11071/3600 = 114.19030853
        assert abs(lon - 114.19030853) < 1e-6
        
        # Test negative sign conversion
        lat_neg = node.dms_to_decimal("-22", "18", "05.61075")
        assert abs(lat_neg - (-22.30155854)) < 1e-6

        node.destroy_node()
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

def test_load_ground_truth(ros_init):
    with tempfile.NamedTemporaryFile(mode='w+', delete=False) as f:
        f.write(MOCK_GT_CONTENT)
        temp_path = f.name

    try:
        node = UrbanNavConverter(parameter_overrides=[
            Parameter('gt_path', Parameter.Type.STRING, temp_path)
        ])
        
        assert len(node.gt_data) == 4
        assert node.start_utc_time == 1621578524.00
        
        # Check specific parsed values
        first_entry = node.gt_data[0]
        assert first_entry['utc_time'] == 1621578524.00
        assert abs(first_entry['lat'] - 22.30155854) < 1e-6
        assert abs(first_entry['lon'] - 114.19030853) < 1e-6
        assert first_entry['alt'] == 2.894
        assert first_entry['vel_forward'] == 1.500
        assert first_entry['heading'] == -59.3645242579
        
        last_entry = node.gt_data[3]
        assert last_entry['utc_time'] == 1621578527.00
        assert last_entry['vel_forward'] == 3.000

        node.destroy_node()
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

def test_imu_callback_and_publishing(ros_init):
    with tempfile.NamedTemporaryFile(mode='w+', delete=False) as f:
        f.write(MOCK_GT_CONTENT)
        temp_path = f.name

    try:
        node = UrbanNavConverter(parameter_overrides=[
            Parameter('gt_path', Parameter.Type.STRING, temp_path),
            Parameter('sim_gps_loss', Parameter.Type.BOOL, True),
            Parameter('gps_loss_start_sec', Parameter.Type.DOUBLE, 1.0),
            Parameter('gps_loss_duration_sec', Parameter.Type.DOUBLE, 2.0),
        ])
        
        # Lists to store published messages
        gps_msgs = []
        odom_msgs = []
        
        gps_sub = node.create_subscription(NavSatFix, '/gps/fix', lambda msg: gps_msgs.append(msg), 10)
        odom_sub = node.create_subscription(Odometry, '/vehicle/odom', lambda msg: odom_msgs.append(msg), 10)
        
        # Allow DDS discovery
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.01)
        
        # Helper to spin the node multiple times to process all queued callbacks
        def spin_some(node, count=5):
            for _ in range(count):
                rclpy.spin_once(node, timeout_sec=0.01)

        # Helper to create IMU messages
        def make_imu(sec, nanosec, omega_z=0.0):
            msg = Imu()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            msg.angular_velocity.z = omega_z
            return msg

        # --- Step 1: Call IMU at 1621578524.1 (Offset 0.1s)
        # Should publish GPS (since last_gps_pub_time = 0) and Odom (since last_odom_pub_time = 0)
        node.imu_callback(make_imu(1621578524, 100000000))
        
        spin_some(node)
        
        assert len(gps_msgs) == 1
        assert len(odom_msgs) == 1
        
        # Verify initial GPS
        assert gps_msgs[0].status.status == 0  # STATUS_FIX (since offset 0.1 is not in loss range [1.0, 3.0])
        assert abs(gps_msgs[0].latitude - 22.30155854) < 1e-6
        assert gps_msgs[0].position_covariance[0] == 1.0
        
        # Verify initial Odom
        assert odom_msgs[0].twist.twist.linear.x == 1.500
        
        # --- Step 2: Call IMU again after 0.05 seconds (1621578524.15)
        # Should publish Odometry but NOT GPS (GPS has 1Hz rate limit)
        node.imu_callback(make_imu(1621578524, 150000000))
        spin_some(node)
        assert len(gps_msgs) == 1
        assert len(odom_msgs) == 2
        
        # --- Step 3: Call IMU at 1621578525.2 (Offset 1.2s -> in GPS loss range [1.0, 3.0])
        # Should publish GPS and Odometry. GPS should have STATUS_NO_FIX (-1) and high covariance.
        node.imu_callback(make_imu(1621578525, 200000000))
        spin_some(node)
        assert len(gps_msgs) == 2
        assert len(odom_msgs) == 3
        
        # Check GPS loss message properties
        lost_gps = gps_msgs[1]
        assert lost_gps.status.status == -1  # STATUS_NO_FIX
        assert lost_gps.position_covariance[0] == 99.0
        
        # Check Odometry matches index 1 (closest to 1.2s -> 1s)
        # Entry at index 1 has vel_forward = 2.0
        assert odom_msgs[2].twist.twist.linear.x == 2.0

        # --- Step 4: Call IMU at 1621578527.5 (Offset 3.5s -> recovered)
        # Should publish GPS and Odometry. GPS should be recovered (STATUS_FIX).
        # Closest ground truth entry is index 3 (offset 3.5s rounded is 4s, bounded to index 3)
        node.imu_callback(make_imu(1621578527, 500000000))
        spin_some(node)
        assert len(gps_msgs) == 3
        assert len(odom_msgs) == 4
        
        recovered_gps = gps_msgs[2]
        assert recovered_gps.status.status == 0  # STATUS_FIX
        assert recovered_gps.position_covariance[0] == 1.0
        assert odom_msgs[3].twist.twist.linear.x == 3.0
        
        node.destroy_node()
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)
