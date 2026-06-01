# coding: utf-8

import cv2
import h5py


path = r'/media/psf/work/data/ost_calibration/imu_to_vpcam/20240113-30_of_120Hz-mercury-1920x1200-32_33-naked-fix-cam2imu_fast-final-test10-imu-intrinsic/0_img_640x400.h5'
cv2.namedWindow('haha')
with h5py.File(path, 'r') as h:
	data = h['images']
	print(data.shape)
	for i in range(len(data)):
		cv2.imshow('haha', data[i].squeeze())
		cv2.waitKey(1)


# import cv2
# import h5py
# import numpy as np

# path = r'/media/psf/work/data/ost_calibration/imu_to_vpcam/20240113-30_of_120Hz-mercury-1920x1200-32_33-naked-fix-cam2imu_fast-final-test10-imu-intrinsic/0_img_1920x1200.h5'
# path2 = r'/media/psf/work/data/ost_calibration/imu_to_vpcam/20240113-30_of_120Hz-mercury-1920x1200-32_33-naked-fix-cam2imu_fast-final-test10-imu-intrinsic/0_img_640x400.h5'
# cv2.namedWindow('haha')
# with h5py.File(path2, 'w') as h2:
# 	with h5py.File(path, 'r') as h:
# 		data = h['images']
# 		print(data.shape)
# 		h2.attrs['width'] = h.attrs['width'] // 3
# 		h2.attrs['height'] = h.attrs['height'] // 3
# 		h2.attrs['channels'] = h.attrs['channels']
# 		h2.attrs['num_images'] = h.attrs['num_images']
# 		dset = h2.create_dataset('images', (len(data), h.attrs['height'] // 3 , h.attrs['width'] // 3), dtype=np.uint8)
# 		for i in range(len(data)):
# 			print(data[i].squeeze().shape)
# 			dset[i] = cv2.resize(data[i], (640, 400))
# 			cv2.imshow('haha', data[i].squeeze())
# 			cv2.waitKey(1)

