import pickle
#from pclpy import pcl
#import pclpy
import pcl
import pcl.pcl_visualization
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


def depthFrametoPC(deviceData):
    #starttime = time.time()
    depth = deviceData['depth']
    infrared = deviceData['infrared']
    rgb = cv2.cvtColor(np.asanyarray(infrared),cv2.COLOR_GRAY2RGB)
    cameraIntrinsics = deviceData['intrinsics']
    poseMat = deviceData['poseMat']
    [height,width] = np.array(depth.shape)
    nx = np.linspace(0, width-1, width)
    ny = np.linspace(0, height-1, height)
    u, v = np.meshgrid(nx, ny)
    x = (u.flatten() - cameraIntrinsics['ppx'])/cameraIntrinsics['fx']
    y = (v.flatten() - cameraIntrinsics['ppy'])/cameraIntrinsics['fy']
    z = depth.flatten() / 1000
    x = np.multiply(x,z)
    y = np.multiply(y,z)


    rgbB = rgb[:,:,0].flatten().astype(int)
    rgbG = rgb[:,:,1].flatten().astype(int)
    rgbR = rgb[:,:,2].flatten().astype(int)
    rgbPC = rgbR<<16|rgbG<<8|rgbB
    points = np.asanyarray([x,y,z])

    n = points.shape[1] 
    points_ = np.vstack((points, np.ones((1,n))))
    points_trans_ = np.matmul(poseMat, points_)
    points_transformed = np.true_divide(points_trans_[:3,:], points_trans_[[-1], :])
    allPoints = np.asanyarray([points_transformed[0,:],points_transformed[1,:],points_transformed[2,:],rgbPC]).T
    return allPoints


def processThread(folder,iteration,full):
    #import pclpy
    #import pcl
    p = multiprocessing.Pool(6)
    visual = pcl.pcl_visualization.CloudViewing()
    signal.signal(signal.SIGINT, signalHandler)
    i=0
    while True:
        if not full:
            i+=10

        script_path = os.path.abspath(__file__) # i.e. /path/to/dir/foobar.py
        scriptDir = os.path.split(script_path)[0]
        print(i)
        fname = folder+'\\'+str(format(int(iteration), '02d'))+'\\'+str(format(i, '05d'))+'.pickle'
        file = open(os.path.join(scriptDir,fname),'rb')
        frame = pickle.load(file)
        file.close()
        #i+=1
        cloud = pcl.PointCloud_PointXYZRGBA()
        listP = [data for serial, data in frame.items()]
        try:
            cameraPoints = p.map(depthFrametoPC, listP)
            allPoints = np.empty((0,4))
            for camera in cameraPoints:
                #camera = depthFrametoPC(item)
                allPoints = np.append(allPoints, camera, axis=0)
            #p.close()
            cv2.imshow('color', frame['822512060522']['infrared'])
            cv2.waitKey(1)
            cloud.from_array(allPoints.astype('float32'))
            visual.ShowColorACloud(cloud)
            i+=1
        except KeyboardInterrupt:
            p.terminate()
        

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", help="folder with data files",
                        nargs='?',default='data')
    parser.add_argument("--iteration", help="iteration of data to process",
                        nargs='?',default=1)
    parser.add_argument("--full", help="playback at full fps",
                        nargs='?', default=0)
    args = parser.parse_args()
    multiprocessing.freeze_support()
    processThread(args.folder,args.iteration,args.full)

