# author: wangye(Wayne)
# license: Apache Licence
# file: change_imu_format_from_taurus_txt_to_euroc_data_csv.py
# time: 2023-12-20-19:31:57
# contact: wang121ye@hotmail.com
# site:  wangyendt@github.com
# software: PyCharm
# code is far away from bugs.


import sys
import os
import pandas as pd
import collections
import numpy as np
import scipy as sp
import re
import argparse
from Utils import Helper
from pywayne.tools import wayne_print, read_yaml_config


parser = argparse.ArgumentParser(description='This script is used for transform data.txt to data.csv')
parser.add_argument('path', type=str, help='path/to/dataset')
parser.add_argument('fixture_id', type=str, help='current calibration fixture id')
parser.add_argument('config_yaml_file', type=str, help='path to main config yaml file')
parser.add_argument('--interpolation', dest='do_interpolation', action='store_true', help='If interpolation variable is set, we do interpolation on data.txt')
if len(sys.argv) == 1:
	parser.print_help()
try:
	parsed = parser.parse_args()
except:
	exit(1)

dataset_root = parsed.path
fixture_id = parsed.fixture_id
config_yaml_file = parsed.config_yaml_file
do_interpolation = parsed.do_interpolation
wayne_print(f'{dataset_root=}', 'yellow', True)
wayne_print(f'{fixture_id=}', 'yellow', True)
wayne_print(f'{do_interpolation=}', 'yellow', True)
wayne_print(f'{config_yaml_file=}', 'yellow', True)

helper = Helper(config_yaml_file)


imu_file = os.path.join(dataset_root, f'data{fixture_id}.txt')
new_imu_file = os.path.join(dataset_root, f'data{fixture_id}.csv')

wayne_print(f'{imu_file=}', 'yellow', True)
if not os.path.exists(imu_file):
	print('no need to transfer from taurus data to euroc data')
	exit(0)


sensor_data = collections.defaultdict(list)
sensor_data_npy = collections.defaultdict(np.array)
# with open(imu_file, 'r', encoding='UTF-16LE', errors='ignore') as f:
with open(imu_file, 'r', encoding='UTF-8', errors='ignore') as f:
	lines = f.readlines()
	wayne_print(f'{len(lines)=}', 'yellow', True)
	for i, line in enumerate(lines):
		line = line.strip()
		if not any(kw in line for kw in ('acc', 'gyro', 'mag')): continue
		if 'get response!!!!65' in line: continue
		sensor, data = re.findall(r'(.*) = (.*)', line)[0]
		sensor_data[sensor].append([float(d) for d in data.split(' ') if d])
wayne_print(f'{np.array(sensor_data["acc"]).shape=}', 'yellow', True)
wayne_print(f'{np.array(sensor_data["gyro"]).shape=}', 'yellow', True)
wayne_print(f'{np.array(sensor_data["mag"]).shape=}', 'yellow', True)
sensor_data_npy['acc'] = np.array(sensor_data['acc'])[:, :3]
sensor_data_npy['gyro'] = np.array(sensor_data['gyro'])[:, :3]
sensor_data_npy['mag'] = np.array(sensor_data['mag'])[:, :3]
N_start = 100
N = min(len(sensor_data_npy['acc']), len(sensor_data_npy['gyro']), len(sensor_data_npy['mag']))
wayne_print(f'{N=}', 'yellow', True)
sensor_data_npy['acc'] = sensor_data_npy['acc'][N_start:N]
sensor_data_npy['gyro'] = sensor_data_npy['gyro'][N_start:N]
sensor_data_npy['mag'] = sensor_data_npy['mag'][N_start:N]
sensor_data_npy['ts'] = np.array(sensor_data['acc'])[:, 3][N_start:N]
wayne_print(f"{sensor_data_npy['ts'][np.where(sensor_data_npy['mag'][:, 2] < -500)]=}", 'yellow', True)
sensor_data_npy['ts'] -= sensor_data_npy['ts'][0]
sensor_data_npy['ts'] /= 1e4
sensor_data_npy['ts'] *= 1e9
sensor_data_npy['ts'] = sensor_data_npy['ts'].astype(int)


ts = sensor_data_npy['ts']
acc = sensor_data_npy['acc']
gyro = sensor_data_npy['gyro']
acc_param_path = helper.get_module_path('imu_intrinsic', f'{fixture_id}_acc', max_waiting_time=1000)
gyro_param_path = helper.get_module_path('imu_intrinsic', f'{fixture_id}_gyro', max_waiting_time=1000)
exp_param_path = os.path.join(os.path.dirname(acc_param_path), 'exp_param.yaml')

def apply_params(acc_param_path, gyro_param_path, ts, acc, gyro):
	wayne_print(f'{acc_param_path=}', 'yellow', True)
	wayne_print(f'{gyro_param_path=}', 'yellow', True)
	with open(acc_param_path, 'r') as f:
		lines = [[float(d) for d in re.split('\s+', l.strip())] for l in map(str.strip, f.readlines()) if l]
		wayne_print(f'{lines=}', 'yellow', True)
		T_a = np.array(lines[:3])
		K_a = np.array(lines[3:6])
		b_a = np.array(lines[6:9])
		wayne_print(f'{T_a=}, {K_a=}, {b_a=}', 'yellow', True)
		wayne_print(f'{T_a.shape=}, {K_a.shape=}, {b_a.shape=}', 'yellow', True)

	with open(gyro_param_path, 'r') as f:
		lines = [[float(d) for d in re.split('\s+', l.strip())] for l in map(str.strip, f.readlines()) if l]
		wayne_print(f'{lines=}', 'yellow', True)
		T_g = np.array(lines[:3])
		K_g = np.array(lines[3:6])
		b_g = np.array(lines[6:9])
		wayne_print(f'{T_g=}, {K_g=}, {b_g=}', 'yellow', True)
		wayne_print(f'{T_g.shape=}, {K_g.shape=}, {b_g.shape=}', 'yellow', True)
	
	cut_duration = [0]
	
	param_mapping = {
		# Accelerometer params
		'Ta_12': lambda v: T_a.__setitem__((0, 1), v),
		'Ta_13': lambda v: T_a.__setitem__((0, 2), v),
		'Ta_23': lambda v: T_a.__setitem__((1, 2), v),
		'Ka_11': lambda v: K_a.__setitem__((0, 0), v),
		'Ka_22': lambda v: K_a.__setitem__((1, 1), v),
		'Ka_33': lambda v: K_a.__setitem__((2, 2), v),
		'ba_1':  lambda v: b_a.__setitem__(0, v),
		'ba_2':  lambda v: b_a.__setitem__(1, v),
		'ba_3':  lambda v: b_a.__setitem__(2, v),
		
		# Gyroscope params
		'Tg_12': lambda v: T_g.__setitem__((0, 1), v),
		'Tg_13': lambda v: T_g.__setitem__((0, 2), v),
		'Tg_21': lambda v: T_g.__setitem__((1, 0), v),
		'Tg_23': lambda v: T_g.__setitem__((1, 2), v),
		'Tg_31': lambda v: T_g.__setitem__((2, 0), v),
		'Tg_32': lambda v: T_g.__setitem__((2, 1), v),
		'Kg_11': lambda v: K_g.__setitem__((0, 0), v),
		'Kg_22': lambda v: K_g.__setitem__((1, 1), v),
		'Kg_33': lambda v: K_g.__setitem__((2, 2), v),
		'bg_1':  lambda v: b_g.__setitem__(0, v),
		'bg_2':  lambda v: b_g.__setitem__(1, v),
		'bg_3':  lambda v: b_g.__setitem__(2, v),

		# imu_data cutting size
		'cut_duration': lambda v: cut_duration.__setitem__(0, v),
	}

	if os.path.exists(exp_param_path):
		exp_param = read_yaml_config(exp_param_path)
		update_count = 0
		for k, v in exp_param.items():
			if k in param_mapping:
				update_var = param_mapping[k]
				update_var(v)
				update_count += 1
				wayne_print(f'{k=}, {v=}, {update_var.__name__}', 'yellow', False)
			else:
				wayne_print(f'{k=} not in param_mapping', 'red', True)
		wayne_print(f'{update_count=} params updated', 'yellow', False)
	else:
		wayne_print(f'{exp_param_path=} not exists', 'red', True)

	acc_calibrated = (T_a @ K_a @ (acc.T - b_a)).T
	gyro_calibrated = (T_g @ K_g @ (gyro.T - b_g)).T
	wayne_print(f'{acc_calibrated.shape=}', 'yellow', False)
	wayne_print(f'{gyro_calibrated.shape=}', 'yellow', False)

	cut_index = np.where(ts <= ts[-1] - cut_duration[0] * 1_000_000_000)
	ts_cut = ts[cut_index]
	acc_calibrated_cut = acc_calibrated[cut_index]
	gyro_calibrated_cut = gyro_calibrated[cut_index]
	wayne_print(f'{ts_cut.shape=}, {acc_calibrated_cut.shape=}, {gyro_calibrated_cut.shape=}', 'yellow', False)

	# import matplotlib.pyplot as plt

	# plt.figure()
	# plt.subplot(121)
	# plt.plot(ts, np.linalg.norm(acc,axis=1))
	# plt.plot(ts, np.linalg.norm(acc_calibrated,axis=1))
	# plt.subplot(322)
	# plt.plot(ts, gyro[:,0])
	# plt.plot(ts, gyro_calibrated[:,0])
	# plt.subplot(324)
	# plt.plot(ts, gyro[:,1])
	# plt.plot(ts, gyro_calibrated[:,1])
	# plt.subplot(326)
	# plt.plot(ts, gyro[:,2])
	# plt.plot(ts, gyro_calibrated[:,2])
	# plt.show()

	# df = pd.DataFrame(np.c_[ts, gyro_calibrated, acc_calibrated], columns=headers)
	# df.to_csv(imu_path.replace('data.csv','data_calibrated.csv'), index=False)
 
	return ts_cut, acc_calibrated_cut, gyro_calibrated_cut


wayne_print('*'*80, 'yellow', True)
ts, acc, gyro = apply_params(acc_param_path, gyro_param_path, ts, acc, gyro)

data = pd.DataFrame(np.c_[ts, gyro, acc])
data.columns = '#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]'.split(',')
data['#timestamp [ns]'] = data['#timestamp [ns]'].astype(int)

if do_interpolation:
	start = ts.min()
	end = ts.max()
	interval_ns = 2000000  # 2ms的间隔，以纳秒为单位
	new_timestamps = np.arange(start, end, interval_ns)
	timestamps = ts
	sensor_values = data[['w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]']].values
	new_sensor_values = np.zeros((len(new_timestamps), sensor_values.shape[1]))
	for i in range(sensor_values.shape[1]):
		interpolator = sp.interpolate.interp1d(timestamps, sensor_values[:, i], kind='linear', fill_value="extrapolate")
		new_sensor_values[:, i] = interpolator(new_timestamps)
	new_data = pd.DataFrame(new_sensor_values, columns=['w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
	new_data.insert(0, '#timestamp [ns]', new_timestamps)
	wayne_print(f'{new_data.head()=}', 'yellow', True)
	new_data.to_csv(new_imu_file, index=False)
else:
	wayne_print(f'{data.head()=}', 'yellow', True)
	data.to_csv(new_imu_file, index=False)
