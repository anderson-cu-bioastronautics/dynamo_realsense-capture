##########################################################################################################################################
##                                               License: Apache 2.0. See LICENSE  file in root directory.	         	                ##
##########################################################################################################################################
"""
View captured 3D scans from DynaMo

Distributed as a module of DynaMo: https://github.com/anderson-cu-bioastronautics/dynamo_realsense-capture
"""
import pickle
try:
    import pcl
    import pcl.pcl_visualization
except:
    pass
import time
import numpy as np
import multiprocessing
import threading
import queue
import cv2
import argparse
import os
import signal
import sys

def signalHandler(signal,frame):
    sys.exit()


def depthFrametoPC(deviceData, **kwargs):
    """
    Function which takes saved depth and color/infrared 2D frames and converts to a colored 3D pointcloud 

    Parameters
    ----------
    deviceData : dict
        Dictionary with depth frame, infrared frame, depth sensor intrinsics, and transformation matrix

    Optional Arguments
    ------------------
    format: str
        Format to define how to save pointcloud color information. Default is 'pcl', alternative is 'rgb'.
    
    Returns
    -------
    pointCloud : (n,4) array
        Array with rows representing each point with columns denoting x,y,z, and PCL visualization color

    """
    depth = deviceData['depth']
    if 'color' in deviceData: #if a color frame was saved in the frame
        rgb = deviceData['color']
    elif 'infrared' in deviceData: #otherwise use the saved infrared frame
        infrared = deviceData['infrared']
        rgb = cv2.cvtColor(np.asanyarray(infrared),cv2.COLOR_GRAY2RGB)
    else:  #creates blank rgb with same dimension as depth if no color or infrared frame saved
        rgb = np.zeros((depth.shape[0], depth.shape[1], 3))


    cameraIntrinsics = deviceData['intrinsics']
    poseMat = deviceData['poseMat']

    ### The following lines of code are taken from the librealsense box_dimensioner_multicam example                                     ###
    ### See LICENSE.librealsense in root directory                                                                                                    ###
    ### https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/box_dimensioner_multicam/helper_functions.py ###
    [height,width] = np.array(depth.shape)
    nx = np.linspace(0, width-1, width)
    ny = np.linspace(0, height-1, height)
    u, v = np.meshgrid(nx, ny)
    x = (u.flatten() - cameraIntrinsics['ppx'])/cameraIntrinsics['fx']
    y = (v.flatten() - cameraIntrinsics['ppy'])/cameraIntrinsics['fy']
    z = depth.flatten() / 1000
    x = np.multiply(x,z)
    y = np.multiply(y,z)
    ###                                                                                                                                  ###

    rgbB = rgb[:,:,0].flatten().astype(int)
    rgbG = rgb[:,:,1].flatten().astype(int)
    rgbR = rgb[:,:,2].flatten().astype(int)
    rgbPC = rgbR<<16|rgbG<<8|rgbB #Convert rgb values into PCL format to be visualized 
    points = np.asanyarray([x,y,z]) 

    points3x4 = np.vstack((points, np.ones((1,points.shape[1])))) #append 1s to the list of points so that we can multiply by a 4x4 matrix
    pointsTransformed = np.matmul(poseMat, points3x4) #muliply by 4x4 transformation matrix
    pointsTransformed = np.true_divide(pointsTransformed[:3,:], pointsTransformed[[-1], :])

    if not kwargs['format'] or kwargs['format'] == 'pcl':
        pointCloud = np.asanyarray([pointsTransformed[0,:],pointsTransformed[1,:],pointsTransformed[2,:],rgbPC]).T #append column of point colors to transformed points
    elif kwargs['format'] ==  'rgb':
        pointCloud = np.asanyarray([pointsTransformed[0,:],pointsTransformed[1,:],pointsTransformed[2,:], rgbR, rgbG, rgbB]).T #append column of point colors to transformed points
    return pointCloud

def getPointCloud(frame):
    """
    Function which allows for the conversion of frame with multiple cameras into a pointcloud

    Parameters
    ----------
    frame : dict
        Dictionary with keys as serial numbers of each connected camera, with frames from each camera stored

    Returns
    -------
    pointCloud : (n,4) array
        Array with rows representing each point with columns denoting x,y,z, and PCL visualization color

    """
    p = multiprocessing.Pool(6) #create a multiprocessing pool to efficiently create pointclouds in parallel
    signal.signal(signal.SIGINT, signalHandler)
    listP = [data for serial, data in frame.items()]
    cameraPoints = p.map(depthFrametoPC, listP, format='pcl') #process point cloud for each camera in a separate thread
    pointCloud = np.empty((0,4))
    for camera in cameraPoints:
        pointCloud = np.append(pointCloud, camera, axis=0)
    return pointCloud


def  viewPointClouds(folderDirectory,full):
    """
    Function which allows for the viewing of pointClouds 
    
    Parameters
    ----------
    folderDirectory : str
        Directory containing .pickle files of saved frames from capture

    full : int
        0 if you wish to view every 10 frames, 1 if you wish to view every frame
    

    """
    p = multiprocessing.Pool(6) #create a multiprocessing pool to efficiently create pointclouds in parallel
    visual = pcl.pcl_visualization.CloudViewing() #handle for PCL point cloud viewer
    signal.signal(signal.SIGINT, signalHandler)
    i=0
    while True:
        if not full:
            i+=10 

        print(i)
        fname = folderDirectory+'\\'+str(format(i, '05d'))+'.pickle'
        file = open(fname,'rb')
        frame = pickle.load(file)
        file.close()
        #i+=1
        cloud = pcl.PointCloud_PointXYZRGBA()
        listP = [data for serial, data in frame.items()]
        try:
            cameraPoints = p.map(depthFrametoPC, listP) #process point cloud for each camera in a separate thread
            allPoints = np.empty((0,4))
            for camera in cameraPoints:
                allPoints = np.append(allPoints, camera, axis=0)
            cloud.from_array(allPoints.astype('float32')) 
            visual.ShowColorACloud(cloud)
            i+=1
        except KeyboardInterrupt:
            p.terminate()
        

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", help="folder with data files",
                        nargs='?',default='data')
    parser.add_argument("--full", help="playback at full fps",
                        nargs='?', default=0)
    args = parser.parse_args()
    multiprocessing.freeze_support()
    viewPointCloud(args.folder,args.full)

