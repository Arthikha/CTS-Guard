# # Using 1D Kalman

# import numpy as np
# import matplotlib.pyplot as plt

# data = np.loadtxt('imu_data.csv', delimiter=',')
# t = np.arange(data.shape[0]) * 0.1  # adjust time step if different
# acc = data[:,0:3]
# gyro = data[:,3:6]
# temp = data[:,6]

# def kalman_filter(z, Q=0.001, R=0.01):
#     x_est = 0.0      # initial estimate
#     P = 1.0          # initial covariance
#     x_estimates = []

#     for measurement in z:
#         # Prediction step
#         x_pred = x_est
#         P_pred = P + Q

#         # Update step
#         K = P_pred / (P_pred + R)
#         x_est = x_pred + K * (measurement - x_pred)
#         P = (1 - K) * P_pred

#         x_estimates.append(x_est)

#     return np.array(x_estimates)

# # Apply to all axes
# acc_x_f = kalman_filter(acc[:,0])
# acc_y_f = kalman_filter(acc[:,1])
# acc_z_f = kalman_filter(acc[:,2])
# gyro_x_f = kalman_filter(gyro[:,0])
# gyro_y_f = kalman_filter(gyro[:,1])
# gyro_z_f = kalman_filter(gyro[:,2])

# plt.figure(figsize=(10,6))
# plt.subplot(2,1,1)
# plt.plot(t, acc[:,0], 'r--', label='Raw Acc X')
# plt.plot(t, acc_x_f, 'r', label='Filtered Acc X')
# plt.plot(t, acc[:,1], 'g--', label='Raw Acc Y')
# plt.plot(t, acc_y_f, 'g', label='Filtered Acc Y')
# plt.plot(t, acc[:,2], 'b--', label='Raw Acc Z')
# plt.plot(t, acc_z_f, 'b', label='Filtered Acc Z')
# plt.title("Accelerometer Kalman Filter")
# plt.xlabel("Time [s]")
# plt.ylabel("m/s²")
# plt.legend()

# plt.subplot(2,1,2)
# plt.plot(t, gyro[:,0], 'r--', label='Raw Gyro X')
# plt.plot(t, gyro_x_f, 'r', label='Filtered Gyro X')
# plt.plot(t, gyro[:,1], 'g--', label='Raw Gyro Y')
# plt.plot(t, gyro_y_f, 'g', label='Filtered Gyro Y')
# plt.plot(t, gyro[:,2], 'b--', label='Raw Gyro Z')
# plt.plot(t, gyro_z_f, 'b', label='Filtered Gyro Z')
# plt.title("Gyroscope Kalman Filter")
# plt.xlabel("Time [s]")
# plt.ylabel("rad/s")
# plt.legend()
# plt.tight_layout()
# plt.show()


import numpy as np
import matplotlib.pyplot as plt

# Load your data (adjust 'imu_data.csv' to your file)
data = np.loadtxt('imu_data.csv', delimiter=',')
t = np.arange(data.shape[0]) * 0.1  # Adjust time step if different (e.g., 100 Hz → 0.01 s)
acc = data[:, 0:3] 
gyro = data[:, 3:6] 
temp = data[:, 6]   

# Step 1: Estimate initial biases (since sensor is stationary)
acc_bias = np.mean(acc, axis=0) 
gyro_bias = np.mean(gyro, axis=0)
expected_acc = np.array([0, 0, 9.8])  
acc_offset = acc_bias - expected_acc  
gyro_offset = gyro_bias  # Gyro should be 0, so bias is the offset

# Step 2: 1D Kalman Filter with tuned parameters
def kalman_filter(z, initial_estimate=0.0, Q=1e-5, R=0.1):
    x_est = initial_estimate  # Start with estimated bias
    P = 1.0  # Initial covariance
    x_estimates = []

    for measurement in z:
        # Prediction step
        x_pred = x_est
        P_pred = P + Q

        # Update step
        K = P_pred / (P_pred + R)
        x_est = x_pred + K * (measurement - x_pred)
        P = (1 - K) * P_pred
        x_estimates.append(x_est)

    return np.array(x_estimates)

# Step 3: Apply Kalman filter to each axis with initial bias estimate
# Tune Q and R based on noise characteristics (adjust these after testing)
# Q: Process noise (small for stationary sensor)
# R: Measurement noise (depends on MPU6050 noise, ~0.05-0.5 typically)
acc_x_f = kalman_filter(acc[:, 0], initial_estimate=acc_bias[0], Q=1e-5, R=0.1) - acc_offset[0]
acc_y_f = kalman_filter(acc[:, 1], initial_estimate=acc_bias[1], Q=1e-5, R=0.1) - acc_offset[1]
acc_z_f = kalman_filter(acc[:, 2], initial_estimate=acc_bias[2], Q=1e-5, R=0.1) - acc_offset[2]
gyro_x_f = kalman_filter(gyro[:, 0], initial_estimate=gyro_bias[0], Q=1e-6, R=0.01) - gyro_offset[0]
gyro_y_f = kalman_filter(gyro[:, 1], initial_estimate=gyro_bias[1], Q=1e-6, R=0.01) - gyro_offset[1]
gyro_z_f = kalman_filter(gyro[:, 2], initial_estimate=gyro_bias[2], Q=1e-6, R=0.01) - gyro_offset[2]

# Step 4: Plot raw vs filtered data
plt.figure(figsize=(10, 8))

# Accelerometer
plt.subplot(2, 1, 1)
plt.plot(t, acc[:, 0], 'r--', label='Raw Acc X')
plt.plot(t, acc_x_f, 'r', label='Filtered Acc X')
plt.plot(t, acc[:, 1], 'g--', label='Raw Acc Y')
plt.plot(t, acc_y_f, 'g', label='Filtered Acc Y')
plt.plot(t, acc[:, 2], 'b--', label='Raw Acc Z')
plt.plot(t, acc_z_f, 'b', label='Filtered Acc Z')
plt.axhline(y=0, color='k', linestyle=':', alpha=0.3)  # Reference for Acc X/Y
plt.axhline(y=9.8, color='b', linestyle=':', alpha=0.3)  # Reference for Acc Z
plt.title("Accelerometer with Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s²]")
plt.legend()

# Gyroscope
plt.subplot(2, 1, 2)
plt.plot(t, gyro[:, 0], 'r--', label='Raw Gyro X')
plt.plot(t, gyro_x_f, 'r', label='Filtered Gyro X')
plt.plot(t, gyro[:, 1], 'g--', label='Raw Gyro Y')
plt.plot(t, gyro_y_f, 'g', label='Filtered Gyro Y')
plt.plot(t, gyro[:, 2], 'b--', label='Raw Gyro Z')
plt.plot(t, gyro_z_f, 'b', label='Filtered Gyro Z')
plt.axhline(y=0, color='k', linestyle=':', alpha=0.3)  # Reference for Gyro
plt.title("Gyroscope with Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Angular Velocity [rad/s]")
plt.legend()

plt.tight_layout()
plt.show()
