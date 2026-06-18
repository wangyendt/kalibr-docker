# coding: utf-8

import h5py
import numpy as np
import cv2
import os

# 路径设置
# image_bin_path = '/ABS/benchmark/imu_to_camera/0_img_.bin'
# hdf5_path = '/ABS/benchmark/imu_to_camera/images.hdf5'
# image_bin_path = r'/ABS/benchmark/imu_to_camera/2023-12-22-60fps-taurus-cam0/0_img_.bin'
# hdf5_path = r'/ABS/benchmark/imu_to_camera/2023-12-22-60fps-taurus-640x400-cam0/0_img_.h5'
# image_bin_path = r'/ABS/benchmark/imu_to_camera/20240103-60Hz-taurus-32_33-wearing-cam2imu_fast_shrink-1/1_img_.bin'
# hdf5_path = r'/ABS/benchmark/imu_to_camera/20240103-60Hz-taurus-32_33-wearing-cam2imu_fast_shrink-1/1_img_640x400.h5'
data_root = r'/ABS/benchmark/imu_to_camera/20240113-120_of_120Hz-mercury-1920x1200-32_33-naked-fix-cam2imu_fast-final-test10'
image_bin_path = os.path.join(data_root, '0_img_.bin')
hdf5_path = os.path.join(data_root, '0_img_1920x1200.h5')

# 图像尺寸和每张图像的大小
w, h, c = 1920, 1200, 1
image_size = w * h * c

# 读取二进制文件
with open(image_bin_path, 'rb') as f:
    data = f.read()
    images = np.frombuffer(data, dtype=np.uint8)
    N = len(data) // image_size
    images = images.reshape((-1, h, w, c))

images = images.squeeze()


resize_to_640x400 = False
if resize_to_640x400:
    w //= 3
    h //= 3
    resized_gray_images = np.empty((images.shape[0], h, w), dtype=np.uint8)

    for i in range(images.shape[0]):
        resized_gray_images[i] = cv2.resize(images[i], (w, h))
    images = resized_gray_images

# # 创建 HDF5 文件
# with h5py.File(hdf5_path, 'w') as hdf5_file:
#     # 创建数据集
#     dset = hdf5_file.create_dataset('images', data=images)

#     # 可选：存储一些元数据
#     hdf5_file.attrs['width'] = w
#     hdf5_file.attrs['height'] = h
#     hdf5_file.attrs['channels'] = c
#     hdf5_file.attrs['num_images'] = N

# 创建 HDF5 文件
with h5py.File(hdf5_path, 'w') as hdf5_file:
    hdf5_file.attrs['width'] = w
    hdf5_file.attrs['height'] = h
    hdf5_file.attrs['channels'] = c
    hdf5_file.attrs['num_images'] = N
    dset = hdf5_file.create_dataset('images', (N, h, w), dtype=np.uint8)
    for i in range(N):
        data = images[i].squeeze()
        dset[i] = data
print(f"图像已保存到HDF5文件: {hdf5_path}")
