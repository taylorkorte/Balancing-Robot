##############  Assignment Requirements: ##############

# Compare angular position using the IMU (raw gyroscope data [integrate]), 
#									 IMU (roll-pitch-yaw), 
#									 and lidar (raw data).

# Compare angular acceleration using the IMU (raw gyroscope data), 
#									 IMU (roll-pitch-yaw [differentiate]) 
#							         and lidar (raw data [differentiate]).

import matplotlib.pyplot as plt
import numpy as np
from scipy import integrate
import pandas as pd
import math

###### IMPORTING/CLEANING AND SEPARATING DATA ######
dataFrame = pd.read_csv('ME134_HW4_SensorData', sep=' ', names = ['time', 'arrow', 'accelx', 'accely', 'accelz', 'gyrox', 'gyroy', 'gyroz', 'lidar'])  

gyroDatax      = dataFrame['gyrox'].to_numpy()
gyroDatay      = dataFrame['gyroy'].to_numpy()
gyroDataz      = dataFrame['gyroz'].to_numpy()

accelDatax      = dataFrame['accelx'].to_numpy()
accelDatay      = dataFrame['accely'].to_numpy()
accelDataz      = dataFrame['accelz'].to_numpy()

lidarDistance   = dataFrame['lidar'].to_numpy()

length,        = gyroDatax.shape

# start time: 14:02:41.740
# end time:   14:02:52.484
# dt:         .1  
time          = np.linspace(0,10.744,num=length)
smalltime     = time[:-1]

# #UNCOMMENT TO PLOT RAW DATA
# plt.figure(1)
# plt.plot(time, gyroDatax)
# plt.plot(time,rollData)
# plt.plot(time,lidarDistance)
# plt.legend(['Gyroscope', 'LIDAR'])
# plt.show()

#################### SMOOTHING ######################

# vector for smoothing x3 = [x1, x2, x3, x4, x5] if window = 5
# e.g data = [1,2,3,4,5,6,7,8,9,10]
# x7 smooth = (5+6+7+8+9)/5

# This function takes in a dataset and window value and outputs the smoothed values
def smoothData(data, window):
	data = list(data)
	mvg_avg = []
	if length % window == 0:
		for i in range(length):
			mvg_avg.append(sum(data[i: i + window]) / window)
		return mvg_avg

	else:
		for i in range(length):
			if i <= length - window:
				temp_avg = sum(data[i: i + window]) / window
			else:
				temp_avg = data[i]
			mvg_avg.append(temp_avg)

		return mvg_avg

#################### DATA ANALYSIS ###################

# finds the angle from the bike to the ground using lidar data
def ConvertLidar(lidarData, bikeHeight,length):

	bikeAngle = np.arcsin(lidarData/(np.sqrt(lidarData**2+bikeHeight**2)))
	bikeAngle = np.degrees(bikeAngle)

	return bikeAngle

# differentiates a set of points
def DifferentiateData(data):

	differentiatedData = np.diff(data)

	return differentiatedData

# integrates a set of points
def IntegratedData(data, time):
	data = list(data)

	integratedData = []
	for i in range(length - 1):
		x = data[i]
		integratedData.append((x+data[i+1])/(time[i+1]-time[i]))
	return integratedData

# calculate roll using acceleration
def FindRoll(accelDatax, accelDatay, accelDataz):
	rollData = np.degrees(np.arctan2(accelDatax,np.sqrt(accelDatay**2+accelDataz**2)))
	return rollData

###################### PLOT DATA ######################

lidarData               = ConvertLidar(lidarDistance, 119, length)

rollData                = FindRoll(accelDatax, accelDatay, accelDataz)

smoothedGyroData        = smoothData(gyroDatax, 3)
smoothedRollData        = smoothData(rollData, 3)
smoothedLidarData       = smoothData(lidarData, 3)

integratedGyroData      = IntegratedData(smoothedGyroData, time)
differentiatedRollData  = DifferentiateData(smoothedRollData)
differentiatedLidarData = DifferentiateData(smoothedLidarData)

## uncomment to check roll and lidar data (should add up to 90 degrees)
#print(lidarData+abs(rollData))

# plot angular position vs time
fig, ax = plt.subplots(3)
# raw gyroscope data [integrate] vs time
ax[0].plot(smalltime,integratedGyroData)
# roll-pitch-yaw vs time
ax[1].plot(time,rollData)
# lidar (raw data) vs time
ax[2].plot(time,lidarData)

ax[0].title.set_text('Integrated Gyro')
ax[1].title.set_text('Roll')
ax[2].title.set_text('Lidar')

plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=None, wspace=0.2, hspace=0.6)
fig.suptitle('Angular Position vs Time', fontsize=16)

plt.show()

# plot angular acceleration vs time
fig2, ax2 = plt.subplots(3)
# raw gyro data vs time
ax2[0].plot(time,gyroDatax)
# differentiated roll data vs time
ax2[1].plot(smalltime,differentiatedRollData)
#differentiated lidar data vs time
ax2[2].plot(smalltime, differentiatedLidarData)

ax2[0].title.set_text('Gyro')
ax2[1].title.set_text('Differentiated Roll')
ax2[2].title.set_text('Differentiated Lidar')

plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=None, wspace=0.2, hspace=0.6)
fig2.suptitle('Angular Acceleration vs Time', fontsize=16)

plt.show()

