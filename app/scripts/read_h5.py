# coding:utf-8


import h5py
import os
import numpy as np
import cv2

# with h5py.File('/ABS/benchmark/imu_to_camera/20240119-60Hz-taurus-1920x1200-32_33-naked-fix-8mm-cam2imu_tiny_board_40_2#')
root = '/ABS/benchmark/imu_to_camera/for_quan'
with h5py.File(os.path.join(root,'0_img_640x400.h5'), 'r') as h:
	N = h.attrs['num_images']
	print(N)
	print(h['images'].shape)
	for i in range(N):
		cv2.imshow('dbg', h['images'][i].squeeze())
		cv2.waitKey(1)