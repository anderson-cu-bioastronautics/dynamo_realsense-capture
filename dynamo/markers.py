##########################################################################################################################################
##                                               License: Apache 2.0. See LICENSE  file in root directory.	         	                ##
##########################################################################################################################################

import numpy as np
import cv2
import pickle

import sklearn
from sklearn.cluster import DBSCAN
import scipy.spatial

from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
import os
import sys

def find_in_view(deviceData):
    """
    Function which returns the location of all detected markers in camera view
    
    Parameters
    ----------
    deviceData : dict
        Dictionary with depth frame, infrared frame, depth sensor intrinsics, and transformation matrix
    
    Returns
    -------
    markers : (n,3) array
        Array with rows representing each marker with columns denoting x,y,z locations
    """
    depth = deviceData['depth']
    if 'infrared' in deviceData: #if infrared frame was saved in frame
        infrared = deviceData['infrared']
    elif 'color' in deviceData: #else convert color frame to black and white
        rgb = deviceData['color']
        infrared = cv2.cvtColor(np.asanyarray(rgb),cv2.COLOR_RGB2GRAY)
    else:  #exit with error
        print("No color or infrared frame saved")
        sys.exit()
    
    markerPoints = np.empty((0,3))
    res,infraredFrameT = cv2.threshold(infraredFrame,150,255,0) #threshold frame to only get bright markers
    contours,heirarchy = cv2.findContours(infraredFrameT,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #identify bright markers as contours

    for cnt in contours: #for each contour
        M = cv2.moments(cnt) #calculate the moment of the contour
        if M["m00"] != 0: #if contour has non-zero moment, calculate the x and y of the center of the cntour
            x = int(M["m10"] / M["m00"]) 
            y = int(M["m01"] / M["m00"])

            z=depthFrame[int(y),int(x)]/1000 #depth value from depth frame
            x1 = (int(x) - cameraIntrinsics['ppx'])/cameraIntrinsics['fx'] #convert x and y to 3D represntation
            y1 = (int(y) - cameraIntrinsics['ppy'])/cameraIntrinsics['fy'] 
            depth = np.array([z*x1,z*y1,z,1])
            depth = np.array(np.matmul(poseMat, depth)[0:3])
            markerPoints=np.vstack((markerPoints,depth))
            
    return markerPoints

def detect(frame):
    """
    Function which detects all makers across all cameras in each frame

    Parameters
    ----------
    frame : dict
        Dictionary with keys as serial numbers of all connected cameras, containing each camera's saved images for the frame

    Returns
    -------
    markers : (n,3) array
        Array with rows representing each marker with columns denoting x,y,z locations
    """
    detectedMarkers = np.empty((0,3))

    for device in frame:
        deviceMarkers = find_in_view(device)
        detectedMarkers = np.append(detectedMarkers, deviceMarkers, axis=0)

    for i in range(0,6): #iterate 6 times
        clusteredMarkers = np.empty((0,3))
        if detectedMarkers.size != 0: #if markers detected in frame
            db = DBSCAN(eps=0.03, min_samples=1,algorithm='ball_tree').fit(detectedMarkers) #fit clustering algorithm
            core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
            core_samples_mask[db.core_sample_indices_] = True
            labels = db.labels_
            for label in set(labels): #isolate markers which are viewed by multiple cameras
                if label != -1:
                    class_member_mask = (labels == label)
                    marker = detectedMarkers[class_member_mask & core_samples_mask]
                    clusteredMarkers = np.vstack((clusteredMarkers, np.mean(marker,axis=0))) #find mean location of markers seen by multiple cameras
            detectedMarkers = clusteredMarkers
    
    return detectedMarkers
