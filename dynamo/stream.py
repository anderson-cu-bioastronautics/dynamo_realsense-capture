##########################################################################################################################################
##                             License: Apache 2.0. See LICENSE file in root directory.		                                            ##
##########################################################################################################################################
"""
Capture a stream of 3D scans from all connected Intel RealSense D4XX cameras

Distributed as a module of DynaMo: https://github.com/anderson-cu-bioastronautics/dynamo_realsense-capture
"""

import pyrealsense2 as rs
import pickle
import queue
import threading
import time
import os
import copy
import numpy as np
from .view import depthFrametoPC

def captureThread(q,deviceManager,stream_time):
    """
    Function to capture frames from connected RealSense Cameras to memory

    Parameters
    ----------
    q : Queue object
        Queue for storing captured frames 

    deviceManager : DeviceManager object
        realsense_device_manager object which manages connections to all cameras

    time : float
        Time in seconds for how long to collect data
    """
    i=0
    fnumber = deviceManager.poll_frames()['822512060853'].get_frame_number() #get frame number to ensure subsequent frames are saved
    while i<int(90*stream_time):
        frames = deviceManager.poll_frames()
        newfnumber = frames['822512060853'].get_frame_number()
        if fnumber != newfnumber: #only save if frame has not already been saved
            q.put(frames)
            i+=1
            print(str(newfnumber)+' '+str(i)+'/'+str(int(90*stream_time)))
        fnumber = newfnumber


def processThread(q,devicesTransformation, saveDirectory):
    """
    Function to process frames from queue to save to disk in pickle format

    Parameters
    ----------
    q : Queue object
        Queue which stores captured frames 

    devicesTransformation : dict
        dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

    folder : str
        Folder name in which to save frames, referenced to current working directory

    iteration : int
        Iteration of data collection, subsequent iterations will be saved in the folder specified

    """


    i=0
    while not q.empty(): #while frames are still waiting to be processed
        framesAll = q.get()
        fname = saveDirectory+'\\'+str(format(i, '05d'))+'.pickle'
        file = open(fname,'wb')
        savedData={}
        for device,frames in framesAll.items():
            deviceData = {}
            align = rs.align(rs.stream.depth) #setup rs.align object
            alignedFrames = align.process(frames) #align frames
            depthFrame = alignedFrames.get_depth_frame() 
            rsIntrinsics = depthFrame.get_profile().as_video_stream_profile().get_intrinsics()
            deviceData['depth'] = copy.deepcopy(np.asanyarray(depthFrame.get_data())) #save depth frame
            try:
                infraredFrame = alignedFrames.get_infrared_frame(1)
                deviceData['infrared'] = copy.deepcopy(np.asanyarray(infraredFrame.get_data())) #save infrared frame
            except:
                pass
            try:
                colorFrame = alignedFrames.get_color_frame()
                deviceData['color'] = copy.deepcopy(np.asanyarray(colorFrame.get_data())) #save infrared frame
            except:
                pass    
            deviceData['intrinsics'] = {'ppx': rsIntrinsics.ppx, 'ppy':rsIntrinsics.ppy, 'fx':rsIntrinsics.fx, 'fy':rsIntrinsics.fy} #save relevant intrinsics
            deviceData['poseMat'] = devicesTransformation[device][0] #save transformation matrix
            savedData[device]=deviceData #save each camera's information in master dictionary for frame
        pickle.dump(copy.deepcopy(savedData),file)
        print('processed'+str(i))
        i+=1
        file.close()
        q.task_done()
    print('queue finished')

def single(deviceManager, devicesTransformation):
    """
    Function to return single, aligned pointcloud from conneted Realsense Cameras

    Parameters
    ----------
    deviceManager : DeviceManager object
        realsense_device_manager object which manages connections to all cameras

    devicesTransformation : dict
        dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

    Returns
    -------
    allPoints : (n,6) array
        array containing list of all points from all connected cameras

    """
    frames = deviceManager.poll_frames()
    allPoints = np.empty((0,6))
    for camera,frame in frames.items():
        deviceData = {}
        align = rs.align(rs.stream.depth) #setup rs.align object
        alignedFrames = align.process(frame) #align frames
        depthFrame = alignedFrames.get_depth_frame() 
        rsIntrinsics = depthFrame.get_profile().as_video_stream_profile().get_intrinsics()
        deviceData['depth'] = copy.deepcopy(np.asanyarray(depthFrame.get_data())) #save depth frame
        try:
            infraredFrame = alignedFrames.get_infrared_frame(1)
            deviceData['infrared'] = copy.deepcopy(np.asanyarray(infraredFrame.get_data())) #save infrared frame
        except:
            pass
        try:
            colorFrame = alignedFrames.get_color_frame()
            deviceData['color'] = copy.deepcopy(np.asanyarray(colorFrame.get_data())) #save infrared frame
        except:
            pass    
        deviceData['intrinsics'] = {'ppx': rsIntrinsics.ppx, 'ppy':rsIntrinsics.ppy, 'fx':rsIntrinsics.fx, 'fy':rsIntrinsics.fy} #save relevant intrinsics
        deviceData['poseMat'] = devicesTransformation[camera][0] #save transformation matrix
        points = depthFrametoPC(deviceData, format='rgb')
        allPoints = np.append(allPoints, points, axis=0)
    return allPoints

def start(deviceManager, deviceTransformations, saveDirectory, stream_time):
    """
    Function to save aligned frames from connected RealSense Cameras to disk for future processing

    Parameters
    ----------
    deviceManager : DeviceManager object
        realsense_device_manager object which manages connections to all cameras

    devicesTransformation : dict
        dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

    folder : str
        Folder name in which to save frames, referenced to current working directory

    iteration : int
        Iteration of data collection, subsequent iterations will be saved in the folder specified

    time : float
        Time in seconds for how long to collect data
    """
    q = queue.Queue(maxsize=0) #queue object to hold incoming frames
    capture = threading.Thread(target=captureThread,args=(q,deviceManager,int(stream_time))) #thread which will capture frames
    process = threading.Thread(target=processThread,args=(q,deviceTransformations,saveDirectory)) #thread which will save frames to disk

    capture.start()
    capture.join()
    time.sleep(int(stream_time)+1) #delay to allow capture thread to finish, this may or may not be needed based on hardware specifications but is recommended to reduce dropped frames
    process.start()
    process.join()
    q.join()
    





    