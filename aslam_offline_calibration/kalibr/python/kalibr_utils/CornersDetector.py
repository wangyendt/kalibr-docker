import aslam_cv as acv
import aslam_cameras_april as acv_april
import kalibr_common as kc
import numpy as np
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed
import os
import pickle
import cv2

class CornersDetector:
    def __init__(self, parsed:dict, helper, showExtraction=False, showReproj=False, imageStepping=False) -> None:
        targetConfig = kc.CalibrationTargetParameters(parsed.target_yaml)
        chain = kc.CameraChainParameters(parsed.chain_yaml)
        chain.printDetails()
        assert chain.numCameras() >= 1
        camConfig = chain.getCameraParameters(0)

        #load the calibration target configuration
        targetParams = targetConfig.getTargetParams()
        targetType = targetConfig.getTargetType()

        # showExtraction = True

        if targetType == 'checkerboard':
            options = acv.CheckerboardOptions() 
            options.filterQuads = True
            options.normalizeImage = True
            options.useAdaptiveThreshold = True        
            options.performFastCheck = False
            options.windowWidth = 5
            options.showExtractionVideo = showExtraction
            grid = acv.GridCalibrationTargetCheckerboard(targetParams['targetRows'], 
                                                            targetParams['targetCols'], 
                                                            targetParams['rowSpacingMeters'], 
                                                            targetParams['colSpacingMeters'],
                                                            options)
        elif targetType == 'circlegrid':
            options = acv.CirclegridOptions()
            options.showExtractionVideo = showExtraction
            options.useAsymmetricCirclegrid = targetParams['asymmetricGrid']
            grid = acv.GridCalibrationTargetCirclegrid(targetParams['targetRows'],
                                                          targetParams['targetCols'], 
                                                          targetParams['spacingMeters'], 
                                                          options)
        elif targetType == 'aprilgrid':
            options = acv_april.AprilgridOptions() 
            options.showExtractionVideo = showExtraction
            options.minTagsForValidObs = int( np.max( [targetParams['tagRows'], targetParams['tagCols']] ) + 1 )
            
            grid = acv_april.GridCalibrationTargetAprilgrid(targetParams['tagRows'],
                                                            targetParams['tagCols'], 
                                                            targetParams['tagSize'], 
                                                            targetParams['tagSpacing'], 
                                                            options)
        else:
            raise RuntimeError( "Unknown calibration target." )
                          
        options = acv.GridDetectorOptions() 
        options.imageStepping = imageStepping
        options.plotCornerReprojection = showReproj
        options.filterCornerOutliers = True
        self.camera = kc.AslamCamera.fromParameters( camConfig )
        self.detector = acv.GridDetector(self.camera.geometry, grid, options)
        self.parsed = parsed

        self.helper = helper

        # self.executor = ProcessPoolExecutor(max(1, os.cpu_count() - 1))
        self.executor = ProcessPoolExecutor(12)
        self.futures = []
    
    @staticmethod
    def process_image(ts, img, packer, detector):
        # return None, ts
        timestamp = packer(ts)
        success, obs = detector.findTarget(timestamp, img)
        # cv2.imshow('haha', img)
        # cv2.waitKey(2)
        if success:
            obs.clearImage()
            # print('findTarget...')
            return obs, ts
        return None, ts

    @staticmethod
    def pack_timestamp(ts):
        ts_in_sec = int(ts // (10 ** 9))
        ts_in_ns = int(ts % (10 ** 9))
        timestamp = acv.Time(ts_in_sec, ts_in_ns)
        return timestamp

    def collect_image_corners(self, sn, ts, img):
        img = cv2.resize(img, (640, 400))
        print(sn, ts, img.shape)
        # for debug
        # self.process_image(ts, img, self.pack_timestamp, self.detector)
        future = self.executor.submit(self.process_image, ts, img, self.pack_timestamp, self.detector)
        self.futures.append(future)
    
    def save_corners_and_timestamp(self):
        targetObservations = []
        timestamps = []
        print('save_corners_and_timestamp...')
        for future in self.futures:
            obs, ts = future.result()
            if obs is not None:
                targetObservations.append(obs)
            timestamps.append(ts)

        extracted_corner_save_file = self.parsed.corner_file[0]
        with open(extracted_corner_save_file, 'wb') as pickle_file:
            pickle.dump(targetObservations, pickle_file)
            print('write pickle completed: ', extracted_corner_save_file)
            # targetObservations = pickle.load(pickle_file)
        timestamp_file = self.parsed.image_timestamp_file[0]
        with open(timestamp_file, 'w') as f:
            f.writelines('\n'.join(map(str, timestamps)))

        for fixture_id in self.helper.get_module_value('all_modules', 'available_fixture_id'):
            if not os.path.exists(timestamp_file):
                self.helper.set_mes_info_by_fixture_id(fixture_id, 'err_msg-top_camera_timestamp_file_not_found', '1')
            if not os.path.exists(extracted_corner_save_file):
                self.helper.set_mes_info_by_fixture_id(fixture_id, 'err_msg-top_camera_corner_file_not_found', '1')

        self.helper.set_module_path('cam_imu', 'top_camera_corners_file', extracted_corner_save_file)
        self.helper.set_module_path('cam_imu', 'top_camera_timestamp_file', timestamp_file)

    def close_pool(self):
        self.executor.shutdown(wait=True)
