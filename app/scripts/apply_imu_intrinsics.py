# coding: utf-8

import numpy as np
import platform
import os
import re
import pandas as pd
import matplotlib.pyplot as plt
from pywayne.dsp import butter_bandpass_filter

prefix_path = '/Users/wayne/Documents' if platform.system() == 'Darwin' else '/media/psf'
imu_path = os.path.join(prefix_path, r'work/data/ost_calibration/imu_to_vpcam/20240324-60Hz-1920x1200-cam2imu_largeboard_80mm_taurus_xgj1_test1/data.csv')
param_path = os.path.join(prefix_path, r'work/project/20240131 imu产线标定/step_2.imu_intrinsic_calibration/results/20240324_艾特讯/imu_tk_arm_calibration/20240324_arm_imu_calibration_xinguangji1_test1')
acc_param_path = os.path.join(param_path, 'test_imu_acc.calib')
gyro_param_path = os.path.join(param_path, 'test_imu_gyro.calib')

imu_data = pd.read_csv(imu_path, delimiter=',')
headers = imu_data.columns
imu_data = imu_data.values
ts = imu_data[:,0]
gyro = imu_data[:,1:4]
acc = imu_data[:,4:7]
# plt.figure()
# plt.subplot(211)
# plt.plot(acc)
# plt.subplot(212)
# plt.plot(gyro)
gyro = butter_bandpass_filter(
	gyro, 2, lo=0.1, fs=500, btype='highpass', realtime=False
)
acc = butter_bandpass_filter(
	acc, 2, hi=100, fs=500, btype='lowpass', realtime=False
)
# plt.subplot(211)
# plt.plot(acc)
# plt.subplot(212)
# plt.plot(gyro)
# plt.show()
# exit()

with open(acc_param_path, 'r') as f:
	lines = [[float(d) for d in re.split('\s+', l.strip())] for l in map(str.strip, f.readlines()) if l]
	print(lines)
	T_a = np.array(lines[:3])
	K_a = np.array(lines[3:6])
	b_a = np.array(lines[6:9])
	print(T_a,K_a,b_a)
	print(T_a.shape,K_a.shape,b_a.shape)

with open(gyro_param_path, 'r') as f:
	lines = [[float(d) for d in re.split('\s+', l.strip())] for l in map(str.strip, f.readlines()) if l]
	print(lines)
	T_g = np.array(lines[:3])
	K_g = np.array(lines[3:6])
	b_g = np.array(lines[6:9])
	print(T_g,K_g,b_g)
	print(T_g.shape,K_g.shape,b_g.shape)

acc_calibrated = (T_a @ K_a @ (acc.T - b_a)).T
gyro_calibrated = (T_g @ K_g @ (gyro.T - b_g)).T
print(acc_calibrated.shape)
print(gyro_calibrated.shape)

plt.plot(ts, np.linalg.norm(acc,axis=1))
plt.plot(ts, np.linalg.norm(acc_calibrated,axis=1))
plt.show()

df = pd.DataFrame(np.c_[ts, gyro_calibrated, acc_calibrated], columns=headers)
df.to_csv(imu_path.replace('data.csv','data_calibrated.csv'), index=False)


