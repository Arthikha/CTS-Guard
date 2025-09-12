import numpy as np
import matplotlib.pyplot as plt

# Load CSV
data = np.loadtxt('imu_data.csv', delimiter=',')

t = np.arange(data.shape[0]) * 0.1  # if logged every 0.1 s
acc = data[:,0:3]
gyro = data[:,3:6]
temp = data[:,6]

# Plot accelerometer
plt.figure(figsize=(10,6))
plt.subplot(3,1,1)
plt.plot(t, acc[:,0], label='Acc X')
plt.plot(t, acc[:,1], label='Acc Y')
plt.plot(t, acc[:,2], label='Acc Z')
plt.ylabel('m/s²')
plt.title('Accelerometer Data')
plt.legend()

# Plot gyro
plt.subplot(3,1,2)
plt.plot(t, gyro[:,0], label='Gyro X')
plt.plot(t, gyro[:,1], label='Gyro Y')
plt.plot(t, gyro[:,2], label='Gyro Z')
plt.ylabel('rad/s')
plt.title('Gyroscope Data')
plt.legend()

# Plot temperature
plt.subplot(3,1,3)
plt.plot(t, temp, label='Temp')
plt.ylabel('°C')
plt.xlabel('Time [s]')
plt.title('Temperature')
plt.legend()

plt.tight_layout()
plt.show()
