import sm

import numpy as np
import sys
import multiprocessing
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import time
import copy
import cv2
import pickle

def multicoreExtractionWrapper(dataset, detector, taskq, resultq, clearImages, noTransformation):    
    if hasattr(dataset, 'reopen'):
        dataset.reopen()
    while 1:
        try:
            task = taskq.get_nowait()
        except queue.Empty:
            return
        idx = task[0]
        stamp, image = dataset.getImage(idx)
        
        if noTransformation:
            success, obs = detector.findTargetNoTransformation(stamp, np.array(image))
        else:
            # ts = time.time()
            success, obs = detector.findTarget(stamp, np.array(image))
            # print(f'{time.time() - ts}s for find Target')
            
        if clearImages:
            obs.clearImage()
        if success:
            resultq.put( (obs, idx) )

def extractCornersFromDataset(dataset, detector, multithreading=False, numProcesses=None, clearImages=True, noTransformation=False):
    print("Extracting calibration target corners")    
    targetObservations = []

    if hasattr(dataset, 'extracted_corner_save_file'):
        with open(dataset.extracted_corner_save_file, 'rb') as pickle_file:
            return pickle.load(pickle_file)

    numImages = dataset.numImages()

    if 'CornerImageDatasetReader' in type(dataset).__name__:
        for timestamp, corner_tags, camera_size in dataset.readDataset():
            data = [[(valid, (px, py)) for direction, (valid, px, py) in sorted(corner_tag.items())] for tag_id, corner_tag in corner_tags.items()]
            success, obs = detector.findTargetCorners(timestamp, data, camera_size['width'], camera_size['height'])
            if success:
                targetObservations.append(obs)
        return targetObservations
    
    # prepare progess bar
    iProgress = sm.Progress2(numImages)
    iProgress.sample()
            
    if multithreading:   
        if not numProcesses:
            numProcesses = max(1,multiprocessing.cpu_count()-1)
        try:      
            manager = multiprocessing.Manager()
            resultq = manager.Queue()
            manager2 = multiprocessing.Manager()
            taskq = manager2.Queue()

            # ts = time.time()
            for idx in range(dataset.numImages()):
                taskq.put((idx,))
            # for idx, (timestamp, image) in enumerate(dataset.readDataset()):
            #     taskq.put( (idx, timestamp, image) )
            # print(f'\033[33mtaskq.put: {time.time() - ts : .3f}s\033[0m')
            # ts = time.time()

            plist=list()
            # numProcesses = 1
            for pidx in range(0, numProcesses):
                detector_copy = copy.copy(detector)
                p = multiprocessing.Process(target=multicoreExtractionWrapper, args=(dataset, detector_copy, taskq, resultq, clearImages, noTransformation, ))
                p.start()
                plist.append(p)
            # print(f'\033[33mplist.append: {time.time() - ts : .3f}s\033[0m')

            ts = time.time()
            #wait for results
            last_done=0
            while 1:
                if all([not p.is_alive() for p in plist]):
                    time.sleep(0.1)
                    break
                done = numImages-taskq.qsize()
                sys.stdout.flush()
                if (done-last_done) > 0:
                    iProgress.sample(done-last_done)
                last_done = done
                time.sleep(0.5)
            resultq.put('STOP')
            print(f'\033[33mresultq.put: {time.time() - ts : .3f}s\033[0m')
            ts = time.time()
        except Exception as e:
            raise RuntimeError("Exception during multithreaded extraction: {0}".format(e))
        
        #get result sorted by time (=idx)
        if resultq.qsize() > 1:
            targetObservations = [[]]*(resultq.qsize()-1)
            for lidx, data in enumerate(iter(resultq.get, 'STOP')):
                obs=data[0]; time_idx = data[1]
                targetObservations[lidx] = (time_idx, obs)
            targetObservations = list(zip(*sorted(targetObservations, key=lambda tup: tup[0])))[1]
        else:
            targetObservations=[]
        print(f'\033[33msort results{time.time() - ts : .3f}s\033[0m')
        ts = time.time()
    
    #single threaded implementation
    else:
        for timestamp, image in dataset.readDataset():
            if noTransformation:
                success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
            else:
                success, observation = detector.findTarget(timestamp, np.array(image))
            if clearImages:
                observation.clearImage()
            if success == 1:
                targetObservations.append(observation)
            iProgress.sample()

    if len(targetObservations) == 0:
        print("\r")
        sm.logFatal("No corners could be extracted for camera {0}! Check the calibration target configuration and dataset.".format(dataset.topic))
    else:    
        print("\r  Extracted corners for %d images (of %d images)                              " % (len(targetObservations), numImages))

    #close all opencv windows that might be open
    cv2.destroyAllWindows()
    
    return targetObservations
