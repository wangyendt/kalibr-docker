import cv_bridge
import cv2
import rosbag
import os
import numpy as np
import pylab as pl
import aslam_cv as acv
import sm
import h5py


class BagImageDatasetReaderIterator(object):
  def __init__(self, dataset, indices=None):
    self.dataset = dataset
    if indices is None:
      self.indices = np.arange(dataset.numImages())
    else:
      self.indices = indices
    self.iter = self.indices.__iter__()

  def __iter__(self):
    return self

  def next(self):
    # required for python 2.x compatibility
    idx = next(self.iter)
    return self.dataset.getImage(idx)

  def __next__(self):
    idx = next(self.iter)
    return self.dataset.getImage(idx)


class BagImageDatasetReader(object):
  def __init__(self, bagfile, imagetopic, bag_from_to=None, perform_synchronization=False, bag_freq=None):
    self.bagfile = bagfile
    self.topic = imagetopic
    self.perform_synchronization = perform_synchronization
    self.bag = rosbag.Bag(bagfile)
    self.uncompress = None
    if imagetopic is None:
      raise RuntimeError(
          "Please pass in a topic name referring to the image stream in the bag file\n{0}".format(self.bag))

    self.CVB = cv_bridge.CvBridge()
    # Get the message indices
    conx = self.bag._get_connections(topics=imagetopic)
    indices = self.bag._get_indexes(conx)

    try:
      self.index = next(indices)
    except:
      raise RuntimeError("Could not find topic {0} in {1}.".format(imagetopic, self.bagfile))

    self.indices = np.arange(len(self.index))

    # sort the indices by header.stamp
    self.indices = self.sortByTime(self.indices)

    # go through the bag and remove the indices outside the timespan [bag_start_time, bag_end_time]
    if bag_from_to:
      self.indices = self.truncateIndicesFromTime(self.indices, bag_from_to)

    # go through and remove indices not at the correct frequency
    if bag_freq:
      self.indices = self.truncateIndicesFromFreq(self.indices, bag_freq)

  def reopen(self):
    try:
      self.bag.close()
    except Exception:
      pass
    self.bag = rosbag.Bag(self.bagfile)
    self.CVB = cv_bridge.CvBridge()
    self.uncompress = None

  # sort the ros messegaes by the header time not message time
  def sortByTime(self, indices):
    self.timestamp_corrector = sm.DoubleTimestampCorrector()
    timestamps = list()
    for idx in self.indices:
      topic, data, stamp = self.bag._read_message(self.index[idx].position)
      timestamp = data.header.stamp.secs * 1e9 + data.header.stamp.nsecs
      timestamps.append(timestamp)
      if self.perform_synchronization:
        self.timestamp_corrector.correctTimestamp(data.header.stamp.to_sec(),
                                                  stamp.to_sec())

    sorted_tuples = sorted(zip(timestamps, indices))
    sorted_indices = [tuple_value[1] for tuple_value in sorted_tuples]
    return sorted_indices

  def truncateIndicesFromTime(self, indices, bag_from_to):
    # get the timestamps
    timestamps = list()
    for idx in self.indices:
      topic, data, stamp = self.bag._read_message(self.index[idx].position)
      timestamp = data.header.stamp.secs + data.header.stamp.nsecs / 1.0e9
      timestamps.append(timestamp)

    bagstart = min(timestamps)
    baglength = max(timestamps) - bagstart

    # some value checking
    if bag_from_to[0] >= bag_from_to[1]:
      raise RuntimeError("Bag start time must be bigger than end time.".format(bag_from_to[0]))
    if bag_from_to[0] < 0.0:
      sm.logWarn("Bag start time of {0} s is smaller 0".format(bag_from_to[0]))
    if bag_from_to[1] > baglength:
      sm.logWarn("Bag end time of {0} s is bigger than the total length of {1} s".format(
          bag_from_to[1], baglength))

    # find the valid timestamps
    valid_indices = []
    for idx, timestamp in enumerate(timestamps):
      if timestamp >= (bagstart + bag_from_to[0]) and timestamp <= (bagstart + bag_from_to[1]):
        valid_indices.append(idx)
    sm.logWarn(
        "BagImageDatasetReader: truncated {0} / {1} images (from-to).".format(len(indices) - len(valid_indices), len(indices)))
    return valid_indices

  def truncateIndicesFromFreq(self, indices, freq):

    # some value checking
    if freq < 0.0:
      raise RuntimeError("Frequency {0} Hz is smaller 0".format(freq))

    # find the valid timestamps
    timestamp_last = -1
    valid_indices = []
    for idx in self.indices:
      topic, data, stamp = self.bag._read_message(self.index[idx].position)
      timestamp = data.header.stamp.secs + data.header.stamp.nsecs / 1.0e9
      if timestamp_last < 0.0:
        timestamp_last = timestamp
        valid_indices.append(idx)
        continue
      if (timestamp - timestamp_last) >= 1.0 / freq:
        timestamp_last = timestamp
        valid_indices.append(idx)
    sm.logWarn(
      "BagImageDatasetReader: truncated {0} / {1} images (frequency)".format(len(indices) - len(valid_indices), len(indices)))
    return valid_indices

  def __iter__(self):
    # Reset the bag reading
    return self.readDataset()

  def readDataset(self):
    return BagImageDatasetReaderIterator(self, self.indices)

  def readDatasetShuffle(self):
    indices = self.indices
    np.random.shuffle(indices)
    return BagImageDatasetReaderIterator(self, indices)

  def numImages(self):
    return len(self.indices)

  def getImage(self, idx):
    topic, data, stamp = self.bag._read_message(self.index[idx].position)
    if self.perform_synchronization:
      timestamp = acv.Time(self.timestamp_corrector.getLocalTime(
          data.header.stamp.to_sec()))
    else:
      timestamp = acv.Time(data.header.stamp.secs,
                           data.header.stamp.nsecs)
    if data._type == 'mv_cameras/ImageSnappyMsg':
      if self.uncompress is None:
        from snappy import uncompress
        self.uncompress = uncompress
      img_data = np.reshape(self.uncompress(np.fromstring(
          data.data, dtype='uint8')), (data.height, data.width), order="C")
    elif data._type == 'sensor_msgs/CompressedImage':
      # compressed images only have either mono or BGR normally (png and jpeg)
      # https://github.com/ros-perception/vision_opencv/blob/906d326c146bd1c6fbccc4cd1268253890ac6e1c/cv_bridge/src/cv_bridge.cpp#L480-L506
      img_data = np.array(self.CVB.compressed_imgmsg_to_cv2(data))
      if len(img_data.shape) > 2 and img_data.shape[2] == 3:
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
    elif data._type == 'sensor_msgs/Image':
      if data.encoding == "16UC1" or data.encoding == "mono16":
        image_16u = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = (image_16u / 256).astype("uint8")
      elif data.encoding == "8UC1" or data.encoding == "mono8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
      elif data.encoding == "8UC3" or data.encoding == "bgr8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
      elif data.encoding == "rgb8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2GRAY)
      elif data.encoding == "8UC4" or data.encoding == "bgra8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BGRA2GRAY)
      # bayes encodings conversions from
      # https://github.com/ros-perception/image_pipeline/blob/6caf51bd4484ae846cd8a199f7a6a4b060c6373a/image_proc/src/libimage_proc/processor.cpp#L70
      elif data.encoding == "bayer_rggb8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_BG2GRAY)
      elif data.encoding == "bayer_bggr8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_RG2GRAY)
      elif data.encoding == "bayer_gbrg8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_GR2GRAY)
      elif data.encoding == "bayer_grbg8":
        img_data = np.array(self.CVB.imgmsg_to_cv2(data))
        img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_GB2GRAY)
      else:
        raise RuntimeError(
            "Unsupported Image Encoding: '{}'\nSupported are: "
            "16UC1 / mono16, 8UC1 / mono8, 8UC3 / rgb8 / bgr8, 8UC4 / bgra8, "
            "bayer_rggb8, bayer_bggr8, bayer_gbrg8, bayer_grbg8".format(data.encoding))
    else:
      raise RuntimeError(
        "Unsupported Image Type: '{}'\nSupported are: "
        "mv_cameras/ImageSnappyMsg, sensor_msgs/CompressedImage, sensor_msgs/Image".format(data._type))
    return (timestamp, img_data)


class H5ImageDatasetReaderIterator:
  def __init__(self, dataset, indices=None):
    self.dataset = dataset
    if indices is None:
      self.indices = np.arange(dataset.numImages())
    else:
      self.indices = indices
    self.iter = self.indices.__iter__()

  def __iter__(self):
    return self

  def next(self):
    # required for python 2.x compatibility
    idx = next(self.iter)
    return self.dataset.getImage(idx)

  def __next__(self):
    idx = next(self.iter)
    return self.dataset.getImage(idx)


class H5ImageDatasetReader:
  def __init__(self, h5file, timestampfile, h5_from_to_in_seconds, h5_freq, perform_synchronization=False):
    self.h5file = h5file
    self.timestampfile = timestampfile
    self.perform_synchronization = perform_synchronization
    self.timestamp = np.loadtxt(self.timestampfile)
    self.topic = '/cam0/image_raw'
    self.indices = np.arange(len(self.timestamp))
    # self.w, self.h, self.c = resolution # 1920, 1200, 1
    # self.size_per_image = self.w * self.h * self.c
    with h5py.File(self.h5file, 'r') as hdf5_file:
      width = hdf5_file.attrs['width']
      height = hdf5_file.attrs['height']
      channels = hdf5_file.attrs['channels']
      self.num_images = hdf5_file.attrs['num_images']
    # with open(self.binfile, 'rb') as f:
    #   f.seek(0, 2)
    #   total_size = f.tell() // np.dtype(np.uint8).itemsize
    #   if total_size % self.size_per_image != 0:
    #     raise ValueError(f'{total_size=} cannot be divided by {self.size_per_image=}')
    #   self.num_images = total_size // self.size_per_image
    #   f.seek(0, 0)
    if len(self.timestamp) != self.num_images:
      raise ValueError(f'{len(self.timestamp)=} and {self.num_images} are not equal!')
    # with open(self.binfile, 'rb') as f:
    #   data = f.read()
    #   self.images = np.frombuffer(data, dtype=np.uint8).reshape((self.num_images, self.h, self.w, self.c))

  def __iter__(self):
    # Reset the bag reading
    return self.readDataset()

  def readDataset(self):
    return H5ImageDatasetReaderIterator(self, self.indices)

  def getImage(self, idx):
    ts = self.timestamp[idx]
    ts_in_sec = int(ts // (10**9))
    ts_in_ns = int(ts % (10**9))
    if self.perform_synchronization:
        timestamp = acv.Time(self.timestamp_corrector.getLocalTime(ts_in_sec))
    else:
        timestamp = acv.Time( ts_in_sec, ts_in_ns )
    # img = self.images[idx].squeeze()
    # return timestamp, img
    # with open(self.binfile, 'rb') as f:
    #   f.seek(idx * self.size_per_image)
    #   data = np.frombuffer(f.read(self.size_per_image), dtype=np.uint8).reshape((self.h, self.w, self.c)).squeeze()
    #   return timestamp, data
    with h5py.File(self.h5file, 'r') as hdf5_file:
      dset = hdf5_file['images']
      data = dset[idx]
      return timestamp, data
  
  def numImages(self):
    return self.num_images




class CornerImageDatasetReaderIterator(object):
  def __init__(self, dataset, indices=None):
    self.dataset = dataset
    if indices is None:
      self.indices = np.arange(dataset.numImages())
    else:
      self.indices = indices
    self.iter = self.indices.__iter__()

  def __iter__(self):
    return self

  def next(self):
    # required for python 2.x compatibility
    idx = next(self.iter)
    return self.dataset.getImage(idx)

  def __next__(self):
    idx = next(self.iter)
    return self.dataset.getImage(idx)


class CornerImageDatasetReader(object):
  def __init__(self, cam_data, camera_size, imagetopic, bag_from_to=None, perform_synchronization=False, bag_freq=None):
    self.cam_data = cam_data
    self.camera_size = camera_size
    self.topic = imagetopic
    self.perform_synchronization = perform_synchronization
    self.uncompress = None
    if imagetopic is None:
      raise RuntimeError("Please pass in a topic name referring to the image stream")

    self.CVB = cv_bridge.CvBridge()
    self.timestamp = np.linspace(0, len(self.cam_data)-1, len(self.cam_data)) / 30
    self.indices = np.arange(len(self.timestamp))
    self.num_images = len(self.indices)

  def __iter__(self):
    return self.readDataset()

  def readDataset(self):
    return CornerImageDatasetReaderIterator(self, self.indices)

  def readDatasetShuffle(self):
    indices = self.indices
    np.random.shuffle(indices)
    return CornerImageDatasetReaderIterator(self, indices)

  def getImage(self, idx):
    single_cam_data = self.cam_data[idx]
    ts = self.timestamp[idx] * 10 ** 9
    ts_in_sec = int(ts // (10 ** 9))
    ts_in_ns = int(ts % (10 ** 9))
    timestamp = acv.Time(ts_in_sec, ts_in_ns)
    return timestamp, single_cam_data, self.camera_size

  def numImages(self):
    return len(self.indices)


class CornersImageDatasetReaderIterator(object):
  def __init__(self, dataset, indices=None):
    self.dataset = dataset
    if indices is None:
      self.indices = np.arange(dataset.numImages())
    else:
      self.indices = indices
    self.iter = self.indices.__iter__()

  def __iter__(self):
    return self

  def next(self):
    # required for python 2.x compatibility
    idx = next(self.iter)
    return self.dataset.getImage(idx)

  def __next__(self):
    idx = next(self.iter)
    return self.dataset.getImage(idx)


class CornersImageDatasetReader(object):
  def __init__(self, corner_file, timestampfile, from_to_in_seconds=None, freq=None, perform_synchronization=False):
    self.timestampfile = timestampfile
    self.extracted_corner_save_file = corner_file
    self.perform_synchronization = perform_synchronization
    self.timestamp = np.loadtxt(self.timestampfile)
    self.topic = '/cam0/image_raw'
    self.indices = np.arange(len(self.timestamp))
    self.num_images = len(self.indices)

  def __iter__(self):
    return self.readDataset()

  def readDataset(self):
    return CornersImageDatasetReaderIterator(self, self.indices)

  def getImage(self, idx):
    ts = self.timestamp[idx]
    ts_in_sec = int(ts // (10**9))
    ts_in_ns = int(ts % (10**9))
    if self.perform_synchronization:
      timestamp = acv.Time(self.timestamp_corrector.getLocalTime(ts_in_sec))
    else:
      timestamp = acv.Time(ts_in_sec, ts_in_ns)
    return timestamp, None

  def numImages(self):
    return self.num_images
