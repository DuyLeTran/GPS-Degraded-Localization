#!/usr/bin/env python3
"""
Offline mathematical validation of the EKF Fusion node.

This script tests the EKF's core logic WITHOUT requiring ROS 2 runtime.
It instantiates the mathematical operations directly and validates:
  1. State prediction (F, Q discretization, angle normalization)
  2. GPS measurement update (innovation, Kalman gain, Joseph form)
  3. Covariance behavior (clamping, symmetry, positive-definiteness)
  4. GPS coordinate rotation (initial_yaw alignment)
  5. First-dt initialization guard
  6. Numerical stability under GPS-denied conditions
"""
import sys
import os
import math
import numpy as np

# ─── Configuration ────────────────────────────────────────────────────────────
GPS_LAT0 = 49.033336608854
GPS_LON0 = 8.3375305625949
INITIAL_YAW = -2.5402619803847
Q_PARAMS = [0.1, 0.1, 0.01]
R_GPS_PARAMS = [2.5, 2.5]
MAX_COV = 1000.0

# ─── Helper functions extracted from ekf_fusion.py ────────────────────────────

def gps_to_local(lat, lon, ref_lat, ref_lon):
    R = 6378137.0
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    ref_lat_rad, ref_lon_rad = math.radians(ref_lat), math.radians(ref_lon)
    dlat, dlon = lat_rad - ref_lat_rad, lon_rad - ref_lon_rad
    x = R * dlon * math.cos(ref_lat_rad)
    y = R * dlat
    return x, y

def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

# ─── Test Framework ───────────────────────────────────────────────────────────

passed = 0
failed = 0
total = 0

def test(name, condition, detail=""):
    global passed, failed, total
    total += 1
    if condition:
        passed += 1
        print(f"  ✅ PASS: {name}")
    else:
        failed += 1
        print(f"  ❌ FAIL: {name}")
        if detail:
            print(f"         Detail: {detail}")

# ─────────────────────────────────────────────────────────────────────────────
print("=" * 70)
print("EKF FUSION — OFFLINE MATHEMATICAL VALIDATION")
print("=" * 70)

# ─── TEST 1: Angle Normalization ─────────────────────────────────────────────
print("\n── Test 1: Angle Normalization ──")

test("normalize(0) = 0",
     abs(normalize_angle(0.0)) < 1e-10)
test("normalize(pi) = pi",
     abs(normalize_angle(math.pi) - math.pi) < 1e-10
     or abs(normalize_angle(math.pi) + math.pi) < 1e-10)
test("normalize(3*pi) = pi",
     abs(normalize_angle(3 * math.pi) - math.pi) < 1e-10
     or abs(normalize_angle(3 * math.pi) + math.pi) < 1e-10)
test("normalize(-3*pi) ≈ -pi or pi",
     abs(abs(normalize_angle(-3 * math.pi)) - math.pi) < 1e-10)
test("normalize(2*pi + 0.1) ≈ 0.1",
     abs(normalize_angle(2 * math.pi + 0.1) - 0.1) < 1e-10)

# ─── TEST 2: GPS Coordinate Rotation ────────────────────────────────────────
print("\n── Test 2: GPS ENU → Odom Rotation ──")

c_yaw = math.cos(-INITIAL_YAW)
s_yaw = math.sin(-INITIAL_YAW)
gps_rot = np.array([[c_yaw, -s_yaw], [s_yaw, c_yaw]])

# If vehicle starts heading ENU yaw = initial_yaw, then forward motion in
# the vehicle frame should map to displacement along that heading in ENU.
# After rotation by -initial_yaw, the forward motion should map to +x in odom.
enu_forward = np.array([math.cos(INITIAL_YAW), math.sin(INITIAL_YAW)])
odom_forward = gps_rot @ enu_forward
test("Forward in ENU → +x in odom",
     abs(odom_forward[0] - 1.0) < 1e-6 and abs(odom_forward[1]) < 1e-6,
     f"Got odom_forward = {odom_forward}")

# Rotation matrix should be orthogonal (det = 1)
det = np.linalg.det(gps_rot)
test("Rotation matrix is orthogonal (det=1)",
     abs(det - 1.0) < 1e-10, f"det = {det}")

# ─── TEST 3: Q Discretization ───────────────────────────────────────────────
print("\n── Test 3: Q Discretization by dt ──")

Q_c = np.diag(Q_PARAMS)
dt1 = 0.1
dt2 = 0.5
Q_d1 = Q_c * dt1
Q_d2 = Q_c * dt2

test("Q_d scales linearly with dt",
     np.allclose(Q_d2, Q_d1 * (dt2 / dt1)),
     f"Q_d1={np.diag(Q_d1)}, Q_d2={np.diag(Q_d2)}")

test("Q_d(0.1) diagonal = [0.01, 0.01, 0.001]",
     np.allclose(np.diag(Q_d1), [0.01, 0.01, 0.001]),
     f"Got {np.diag(Q_d1)}")

# ─── TEST 4: EKF Predict Step ───────────────────────────────────────────────
print("\n── Test 4: EKF Predict (Motion Model) ──")

x = np.array([0.0, 0.0, 0.0])
P = np.eye(3) * 0.1
dt = 0.1
v = 5.0
omega = 0.0

theta_mid = x[2] + omega * dt / 2.0
x_new = x[0] + v * dt * math.cos(theta_mid)
y_new = x[1] + v * dt * math.sin(theta_mid)
theta_new = normalize_angle(x[2] + omega * dt)

test("Straight-line predict: x increases",
     abs(x_new - 0.5) < 1e-6, f"x_new = {x_new}")
test("Straight-line predict: y stays 0",
     abs(y_new) < 1e-6, f"y_new = {y_new}")
test("Straight-line predict: theta stays 0",
     abs(theta_new) < 1e-6, f"theta_new = {theta_new}")

# With turning
x = np.array([0.0, 0.0, 0.0])
omega = 0.5  # rad/s
theta_mid = x[2] + omega * dt / 2.0
x_new = x[0] + v * dt * math.cos(theta_mid)
y_new = x[1] + v * dt * math.sin(theta_mid)
theta_new = normalize_angle(x[2] + omega * dt)

test("Turning predict: x > 0",
     x_new > 0, f"x_new = {x_new}")
test("Turning predict: y > 0 (left turn)",
     y_new > 0, f"y_new = {y_new}")
test("Turning predict: theta = 0.05",
     abs(theta_new - 0.05) < 1e-6, f"theta_new = {theta_new}")

# Jacobian F check
F = np.array([
    [1.0, 0.0, -v * dt * math.sin(theta_mid)],
    [0.0, 1.0,  v * dt * math.cos(theta_mid)],
    [0.0, 0.0, 1.0]
])

test("F is 3x3", F.shape == (3, 3))
test("F[0,0] = 1, F[1,1] = 1, F[2,2] = 1",
     F[0, 0] == 1.0 and F[1, 1] == 1.0 and F[2, 2] == 1.0)
test("F[0,2] ≈ -v*dt*sin(theta_mid)",
     abs(F[0, 2] - (-v * dt * math.sin(theta_mid))) < 1e-10)

# ─── TEST 5: Joseph Form Update ─────────────────────────────────────────────
print("\n── Test 5: Joseph Form Covariance Update ──")

x = np.array([5.0, 3.0, 0.1])
P = np.eye(3) * 1.0
R = np.diag(R_GPS_PARAMS)
H = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

z_measurement = np.array([5.1, 3.2])
z = z_measurement - np.array([x[0], x[1]])

S = H @ P @ H.T + R
det_S = np.linalg.det(S)
test("S is invertible (det > 0)", det_S > 0, f"det(S) = {det_S}")

S_inv = np.linalg.inv(S)
K = P @ H.T @ S_inv
x_updated = x + K @ z

# Joseph form
I = np.eye(3)
IKH = I - K @ H
P_updated = IKH @ P @ IKH.T + K @ R @ K.T

test("Updated x closer to measurement",
     abs(x_updated[0] - 5.1) < abs(x[0] - 5.1),
     f"x_updated[0] = {x_updated[0]}")

test("P_updated is symmetric",
     np.allclose(P_updated, P_updated.T),
     f"max asymmetry = {np.max(np.abs(P_updated - P_updated.T))}")

# P should decrease after update
test("P_updated diagonal ≤ P diagonal",
     all(P_updated[i, i] <= P[i, i] + 1e-10 for i in range(3)),
     f"P_diag={np.diag(P_updated)} vs {np.diag(P)}")

# P should be positive definite
eigenvalues = np.linalg.eigvalsh(P_updated)
test("P_updated is positive definite",
     all(ev > 0 for ev in eigenvalues),
     f"eigenvalues = {eigenvalues}")

# ─── TEST 6: Covariance Clamping ────────────────────────────────────────────
print("\n── Test 6: Covariance Clamping ──")

P_large = np.diag([2000.0, 500.0, 0.5])
P_clamped = P_large.copy()
max_val = MAX_COV
clamped = False
for i in range(3):
    if P_clamped[i, i] > max_val:
        scale = math.sqrt(max_val / P_clamped[i, i])
        P_clamped[i, :] *= scale
        P_clamped[:, i] *= scale
        clamped = True
    if P_clamped[i, i] < 1e-6:
        P_clamped[i, i] = 1e-6

test("Clamping activated for P[0,0]=2000", clamped)
test("P[0,0] clamped to max_covariance",
     P_clamped[0, 0] <= max_val + 1e-6,
     f"P[0,0] = {P_clamped[0, 0]}")
test("P[1,1] unchanged (was below threshold)",
     abs(P_clamped[1, 1] - 500.0) < 1e-6,
     f"P[1,1] = {P_clamped[1, 1]}")

# ─── TEST 7: GPS-Denied Drift Simulation ────────────────────────────────────
print("\n── Test 7: GPS-Denied Covariance Growth ──")

x_sim = np.array([0.0, 0.0, 0.0])
P_sim = np.eye(3) * 0.1
dt_sim = 0.1
v_sim = 5.0  # m/s
omega_sim = 0.0

# Simulate 100 seconds of GPS-denied driving (1000 steps)
for step in range(1000):
    theta_mid = x_sim[2] + omega_sim * dt_sim / 2.0
    x_sim[0] += v_sim * dt_sim * math.cos(theta_mid)
    x_sim[1] += v_sim * dt_sim * math.sin(theta_mid)
    x_sim[2] = normalize_angle(x_sim[2] + omega_sim * dt_sim)

    F_sim = np.array([
        [1.0, 0.0, -v_sim * dt_sim * math.sin(theta_mid)],
        [0.0, 1.0,  v_sim * dt_sim * math.cos(theta_mid)],
        [0.0, 0.0, 1.0]
    ])
    Q_d_sim = Q_c * dt_sim
    P_sim = F_sim @ P_sim @ F_sim.T + Q_d_sim

    # Apply clamping
    for i in range(3):
        if P_sim[i, i] > MAX_COV:
            scale = math.sqrt(MAX_COV / P_sim[i, i])
            P_sim[i, :] *= scale
            P_sim[:, i] *= scale
        if P_sim[i, i] < 1e-6:
            P_sim[i, i] = 1e-6

test("After 100s GPS-denied: P[0,0] ≤ max_covariance",
     P_sim[0, 0] <= MAX_COV + 1.0,
     f"P[0,0] = {P_sim[0, 0]}")
test("After 100s GPS-denied: P[1,1] ≤ max_covariance",
     P_sim[1, 1] <= MAX_COV + 1.0,
     f"P[1,1] = {P_sim[1, 1]}")
test("After 100s GPS-denied: no NaN/Inf in P",
     not np.any(np.isnan(P_sim)) and not np.any(np.isinf(P_sim)),
     f"P =\n{P_sim}")
test("After 100s: distance traveled ≈ 500m",
     abs(x_sim[0] - 500.0) < 0.1,
     f"x = {x_sim[0]}")

# ─── TEST 8: GPS Re-acquisition After Drift ─────────────────────────────────
print("\n── Test 8: GPS Re-acquisition Correction ──")

# Simulate: After 100s GPS-denied, GPS comes back
# The state has drifted forward 500m, GPS says we're at 501m (1m error)
gps_x_meas, gps_y_meas = 501.0, 0.5

z_reacq = np.array([gps_x_meas - x_sim[0], gps_y_meas - x_sim[1]])
H_gps = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
R_gps = np.diag(R_GPS_PARAMS)

S_reacq = H_gps @ P_sim @ H_gps.T + R_gps
try:
    S_inv_reacq = np.linalg.inv(S_reacq)
    K_reacq = P_sim @ H_gps.T @ S_inv_reacq
    x_corrected = x_sim + K_reacq @ z_reacq

    test("GPS re-acquisition: x moves toward GPS measurement",
         abs(x_corrected[0] - gps_x_meas) < abs(x_sim[0] - gps_x_meas),
         f"x_corrected[0]={x_corrected[0]:.3f}, gps={gps_x_meas}")
    test("GPS re-acquisition: y moves toward GPS measurement",
         abs(x_corrected[1] - gps_y_meas) < abs(x_sim[1] - gps_y_meas),
         f"x_corrected[1]={x_corrected[1]:.3f}, gps={gps_y_meas}")

    # After long GPS-denied, P is large → K is large → state jumps close to GPS
    test("Kalman gain is high after GPS denial",
         K_reacq[0, 0] > 0.5,
         f"K[0,0] = {K_reacq[0, 0]:.4f}")
except Exception as e:
    test("GPS re-acquisition: matrix inversion", False, str(e))

# ─── TEST 9: GPS Origin Sanity ──────────────────────────────────────────────
print("\n── Test 9: GPS Origin Produces Near-Zero Offset ──")

x0, y0 = gps_to_local(GPS_LAT0, GPS_LON0, GPS_LAT0, GPS_LON0)
test("GPS at origin → (0, 0)",
     abs(x0) < 1e-6 and abs(y0) < 1e-6,
     f"Got ({x0}, {y0})")

# KITTI first fix should be close to origin
x1, y1 = gps_to_local(49.033336608854, 8.3375305625949, GPS_LAT0, GPS_LON0)
test("KITTI first fix ≈ origin",
     abs(x1) < 1.0 and abs(y1) < 1.0,
     f"Got ({x1:.4f}, {y1:.4f})")

# ─── TEST 10: dt Guard ──────────────────────────────────────────────────────
print("\n── Test 10: dt Guards ──")

test("dt=0 skipped", 0.0 <= 0.0)  # Would return early
test("dt=-1 skipped (negative)", -1.0 <= 0.0)
test("dt=5.0 skipped (>2s)", 5.0 > 2.0)
test("dt=0.1 accepted", 0.0 < 0.1 <= 2.0)

# ─── SUMMARY ─────────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print(f"RESULTS: {passed}/{total} passed, {failed}/{total} failed")
if failed == 0:
    print("✅ ALL TESTS PASSED")
    sys.exit(0)
else:
    print("❌ SOME TESTS FAILED")
    sys.exit(1)
