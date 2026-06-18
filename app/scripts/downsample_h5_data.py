# coding: utf-8

import h5py
import os
import re
import numpy as np
import shutil
import pandas as pd

source_dir = '/ABS/benchmark/imu_to_camera/20240113-120Hz-mercury-1920x1200-32_33-naked-fix-cam2imu_fast-final-test12'
source_dir = '/ABS/benchmark/imu_to_camera/20240113-120_of_120Hz-mercury-1920x1200-32_33-naked-fix-cam2imu_fast-final-test10'
source_freq = re.findall('-(.*?)_of.*?Hz-', os.path.basename(source_dir))[0]
# print(source_freq)
target_freq = [30, 60, 90, 120, 150]
print(f'{source_dir=}')

for freq in target_freq:
	if int(source_freq) <= freq:
		continue
	if int(source_freq) % freq != 0:
		print(f'source freq {source_freq} cannot be deviced by freq: {freq}')
		continue
	print(f'{source_freq=}, {freq=}')
	step = int(source_freq) // freq

	target_dir = re.sub('-(.*?)Hz-', f'-{freq}_of_{source_freq}Hz-', source_dir)
	print(f'{target_dir=}')
	os.makedirs(target_dir, exist_ok=True)

	img_timestamp_0_path = os.path.join(source_dir, '0_save_timestamp.txt')
	img_timestamp_0_data = pd.read_csv(img_timestamp_0_path, header=None)[::step]
	img_timestamp_0_data.to_csv(os.path.join(target_dir, '0_save_timestamp.txt'), header=None, index=False)
	print(pd.read_csv(img_timestamp_0_path, header=None).shape)
	print(pd.read_csv(img_timestamp_0_path, header=None)[::step].shape)

	img_timestamp_1_path = os.path.join(source_dir, '1_save_timestamp.txt')
	if os.path.exists(img_timestamp_1_path):
		img_timestamp_1_data = pd.read_csv(img_timestamp_1_path, header=None)[::step]
		img_timestamp_1_data.to_csv(os.path.join(target_dir, '1_save_timestamp.txt'), header=None, index=False)
		print(pd.read_csv(img_timestamp_1_path, header=None).shape)
		print(pd.read_csv(img_timestamp_1_path, header=None)[::step].shape)

	shutil.copy2(
		os.path.join(source_dir, 'data.csv'),
		os.path.join(target_dir, 'data.csv'),
	)
	# for h5_file in ('0_img_640x400.h5', '1_img_640x400.h5'):
	for h5_file in ('0_img_1920x1200.h5',):#, '1_img_640x400.h5'):
		source_h5_file = os.path.join(source_dir, h5_file)
		if not os.path.exists(source_h5_file): continue
		print(f'{source_h5_file=}')
		with h5py.File(source_h5_file, 'r') as h_src:
			images = h_src['images']
			N = h_src.attrs['num_images']
			h = h_src.attrs['height']
			w = h_src.attrs['width']
			c = h_src.attrs['channels']
			with h5py.File(os.path.join(target_dir, h5_file), 'w') as h_dst:
				h_dst.attrs['width'] = w
				h_dst.attrs['height'] = h
				h_dst.attrs['channels'] = c
				h_dst.attrs['num_images'] = (N+step-1) // step
				print(f'{w=},{h=},{c=},{N=},{(N+step-1)//step=},{step=}')
				dset = h_dst.create_dataset('images', ((N+step-1)//step, h, w), dtype=np.uint8)
				cnt = 0
				for i in range(0, N, step):
					cnt += 1
					data = images[i].squeeze()
					dset[(i+1) // step] = data
					# print(i, i//step)
				print(f'{cnt=}')

