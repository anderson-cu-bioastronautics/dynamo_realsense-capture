import pyrealsense2 as rs
import cv2
import numpy as np 
import time
from helperfunctions.realsense_device_manager import DeviceManager
import helperfunctions.calculate_rmsd_kabsch as rmsd
#from plyfile import PlyData, PlyElement

import copy
import threading
import sys
import multiprocessing 
import pickle
import queue

from pclpy import pcl
import pcl
import pcl.pcl_visualization

import argparse

class Calibrate():

    def __init__(self, device_manager,load,new):
        self.deviceManager = device_manager  
        #for frame in range(30): #disposes 30 frames to stabilize autoexposure
        time.sleep(1) #allow for auto-exposure to stabilize and frames to start coming in 
        frames = self.deviceManager.poll_frames2()
        self.devicesIntrinsics = self.deviceManager.get_device_intrinsics(frames)
        if load:
            file = open(load,'rb')
            self.devicesTransformation = pickle.load(file)
        elif new:
            file = open(new+'.cal','wb')
            self.devicesChessboardLocations = {}
            self.devicesTransformation = {}
            
            #Define Calibration Target
            self.chessboardHeight = 4 #3#6 #squares
            self.chessboardWidth = 5 #5#9 #squares
            self.chessboardSquareSize = 0.0253 #0.049 #0.0253 #m
            
            #Perform Calibration steps
            time.sleep(1)
            self.detectChessboard()
            self.poseTransformation()
            pickle.dump(self.devicesTransformation, file)
            file.close()

        #return self.devicesTransformation


    def detectChessboard(self):
        chessboardDeviceCount = 0
        while len(self.devicesChessboardLocations) < len(self.deviceManager._available_devices):
            cameraFrames = self.deviceManager.poll_frames(raw=True)
            for device, frames in cameraFrames.items(): #this will iterate through each device's serial number (which are used as keys in the frames object)
                if not device in self.devicesChessboardLocations:
                    #cameraFrames = self.deviceManager.poll_frames(raw=True)
                    align = rs.align(rs.stream.depth)
                    alignedFrames = align.process(frames)
                    colorImage = np.asanyarray(alignedFrames.get_color_frame().get_data())
                    colorImage2 = np.asanyarray(frames.get_color_frame().get_data())
                    bwImage = cv2.cvtColor(colorImage,cv2.COLOR_BGR2GRAY)
                    bwImage2 = cv2.cvtColor(colorImage2, cv2.COLOR_BGR2GRAY)
                    depthFrame = alignedFrames.get_depth_frame()
                    depthIntrinsics = self.devicesIntrinsics[device][rs.stream.depth]                
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    chessboardFound, corners = cv2.findChessboardCorners(bwImage, (self.chessboardWidth, self.chessboardHeight))
                    if chessboardFound:
                        
                        print(device," sees the chessboard!")
                        points2D = cv2.cornerSubPix(bwImage, corners, (11,11), (-1,-1), criteria)
                        cv2.drawChessboardCorners(bwImage, (self.chessboardWidth, self.chessboardHeight), points2D, chessboardFound)
                        cv2.imshow(device,np.hstack((bwImage,bwImage2)))
                        cv2.waitKey(50)
                        points2D = np.transpose(corners, (2,0,1))
                        points3D = np.zeros((3, len(points2D[0])))
                        validPoints = [False] * len(points2D[0])
                        for index in range(len(points2D[0])):
                            corner = points2D[:,index].flatten()
                            depth = depthFrame.as_depth_frame().get_distance(round(corner[0]), round(corner[1])) #this gets the depth at the pixel
                            if depth != 0 and depth is not None:
                                validPoints[index] = True #sets points which have a depth value as valid
                                points3D[0, index] = (corner[0]-depthIntrinsics.ppx)/depthIntrinsics.fx*depth
                                points3D[1, index] = (corner[1]-depthIntrinsics.ppy)/depthIntrinsics.fy*depth
                                points3D[2, index] = depth
                        self.devicesChessboardLocations[device] = corners, points2D, points3D, validPoints
                        chessboardDeviceCount += 1
                    if not chessboardFound:
                        chessboardDeviceCount = 0
                        cv2.imshow(device,np.hstack((bwImage,bwImage2)))
                        cv2.waitKey(50)
                        print(device," cannot detect the chessboard!")
            time.sleep(1)
            cv2.destroyAllWindows()
            


    def poseTransformation(self):
        for (serial, [corners, points2D, points3D, validPoints]) in self.devicesChessboardLocations.items():
            if len(points2D[0])<5:
                print(serial, " does not have enough points to have a valid depth for calculating the transformatoin")
            else:
                chessboardPoints = np.zeros((self.chessboardWidth*self.chessboardHeight,3), np.float32) #container for 3d coordinates of chessboard corners in global coordinates of chessboard
                chessboardPoints[:,:2] = np.mgrid[0:self.chessboardWidth, 0:self.chessboardHeight].T.reshape(-1,2)
                chessboardPoints = chessboardPoints.transpose() * self.chessboardSquareSize
                validchessboardPoints = chessboardPoints[:,validPoints].transpose()
                validobservedchessboardPoints = points3D[:, validPoints].transpose()

                chessboardPointsCentered = validchessboardPoints - rmsd.centroid(validchessboardPoints)
                observedchessboardCentered = validobservedchessboardPoints - rmsd.centroid(validobservedchessboardPoints)
                rotationMatrix = rmsd.kabsch(chessboardPointsCentered, observedchessboardCentered)
                rmsdValue = rmsd.kabsch_rmsd(chessboardPointsCentered, observedchessboardCentered)
                #translationVector = rmsd.centroid(observedchessboardCentered) - np.matmul(rmsd.centroid(chessboardPointsCentered), rotationMatrix)
                translationVector = rmsd.centroid(validobservedchessboardPoints) - np.matmul(rmsd.centroid(validchessboardPoints), rotationMatrix)
                #rotationMatrix = rotationMatrix.transpose()
                trans = -np.matmul(rotationMatrix, translationVector.transpose())
                poseMat = np.zeros((4,4))
                poseMat[:3,:3] = rotationMatrix
                poseMat[:3,3] = trans.flatten()
                poseMat[3,3] = 1
                self.devicesTransformation[serial] = [poseMat, rmsdValue]

    
class AlignedData():
    def __init__(self, devicesTransformation, devicesIntrisics, deviceManager,fileName):
        self.devicesTransformation = devicesTransformation
        self.deviceManager = deviceManager
        self.devicesIntrinsics = devicesIntrisics
        self.fileName = fileName
        self.stream()
        

    def depthFrametoPC(self, depth, rgb, cameraIntrinsics, poseMat):
        starttime = time.time()
        [height,width] = np.array(depth.shape)
        #[height, width] = [depthFrame.get_height(), depthFrame.get_width()]
        #depth = np.asanyarray(depthFrame.get_data())
        #rgb = np.asanyarray(colorFrame.get_data())
        ##time1 = time.time()
        nx = np.linspace(0, width-1, width)
        ny = np.linspace(0, height-1, height)
        u, v = np.meshgrid(nx, ny)
        x = (u.flatten() - cameraIntrinsics.ppx)/cameraIntrinsics.fx
        y = (v.flatten() - cameraIntrinsics.ppy)/cameraIntrinsics.fy
        z = depth.flatten() / 1000
        x = np.multiply(x,z)
        y = np.multiply(y,z)
        ##time2 = time.time()
        #x = x[np.nonzero(z)]
        #y = y[np.nonzero(z)]
        #z = z[np.nonzero(z)]

        rgbB = rgb[:,:,0].flatten().astype(int)
        rgbG = rgb[:,:,1].flatten().astype(int)
        rgbR = rgb[:,:,2].flatten().astype(int)
        rgbPC = rgbR<<16|rgbG<<8|rgbB
        ##time3=time.time()
        points = np.asanyarray([x,y,z])

        n = points.shape[1] 
        points_ = np.vstack((points, np.ones((1,n))))
        points_trans_ = np.matmul(poseMat, points_)
        points_transformed = np.true_divide(points_trans_[:3,:], points_trans_[[-1], :])
        ##time4=time.time()
        allPoints = np.asanyarray([points_transformed[0,:],points_transformed[1,:],points_transformed[2,:],rgbPC]).T
        ##times = np.array([time1,time2,time3,time4])-starttime
        ##print(times)
        return allPoints


    def captureThread(self,q,deviceManager):
        i=0
        fnumber = deviceManager.poll_frames(raw=True)['822512060853'].get_frame_number()
        while i<30:
            #cloud = pcl.PointCloud_PointXYZRGBA()
            #savedData={}
            #timeStamp = str(time.time())
            frames = deviceManager.poll_frames(raw=True)
            newfnumber = frames['822512060853'].get_frame_number()
            if fnumber != newfnumber:
                q.put(frames)
                #time.sleep(1/110)
                i+=1
                print(str(newfnumber))
            #print(str(i)+':' + str(framesAll['822512060522'].get_frame_number()))
            fnumber = newfnumber


    def processThread(self,q,fileName):
        file = open('dataStore'+str(fileName)+'.pickle','wb')
        align = rs.align(rs.stream.depth)
        #temporalFilter = rs.temporal_filter()
        #temporalFilter.set_option(rs.option.filter_smooth_alpha, 0.26)
        #temporalFilter.set_option(rs.option.filter_smooth_delta, 20)
        i=0
        while not q.empty():
            framesAll = q.get()
            savedData={}
            for device,frames in framesAll.items():
                deviceData = {}
                align = rs.align(rs.stream.depth)
                alignedFrames = align.process(frames)
                #depthFrame = temporalFilter.process(alignedFrames.get_depth_frame())
                depthFrame = alignedFrames.get_depth_frame()
                infraredFrame = alignedFrames.get_infrared_frame(1)
                #colorFrame = cv2.cvtColor(np.asanyarray(infraredFrame.get_data()),cv2.COLOR_GRAY2RGB)
                #depthFrame = alignedFrames.get_depth_frame()
                rsIntrinsics = depthFrame.get_profile().as_video_stream_profile().get_intrinsics()
                #colorFrame = frames.get_color_frame()
                deviceData['depth'] = copy.deepcopy(np.asanyarray(depthFrame.get_data()))
                deviceData['infrared'] = copy.deepcopy(np.asanyarray(infraredFrame.get_data()))
                deviceData['intrinsics'] = {'ppx': rsIntrinsics.ppx, 'ppy':rsIntrinsics.ppy, 'fx':rsIntrinsics.fx, 'fy':rsIntrinsics.fy}
                deviceData['poseMat'] = self.devicesTransformation[device][0]
                savedData[device]=deviceData
            pickle.dump(copy.deepcopy(savedData),file)
            #time.sleep(1/30)
            print('processed'+str(i))
            i+=1
            q.task_done()
        print('queue finished')
        file.close()

    def stream(self):
        q = queue.Queue(maxsize=0)
        worker1 = threading.Thread(target=self.captureThread,args=(q,deviceManager))
        worker2 = threading.Thread(target=self.processThread,args=(q,self.fileName))
        #worker2 = Process(target=self.processThread,args=(q,))
        worker1.start()
        time.sleep(5.5)
        worker2.start()
        worker2.join()
        q.join()

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--load", help="load calibration",
                        nargs='?')
    parser.add_argument("--new", help="new calibration",
                        nargs='?',default='new')
    args = parser.parse_args()
    rsConfig = rs.config()
    resolutionWidth = 848
    resolutionHeight = 480
    frameRate = 30
    rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
    #rsConfig.enable_stream(rs.stream.infrared, 1, resolutionWidth, resolutionHeight, rs.format.y8, frameRate)
    rsConfig.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)

    deviceManager = DeviceManager(rs.context(), rsConfig)
    deviceManager.enable_all_devices(enable_ir_emitter=True)
    deviceManager.load_settings_json('DisparityShift6.json')
    deviceCalibration = Calibrate(deviceManager,args.load,args.new)
    transformation = deviceCalibration.devicesTransformation
    intrinsics = deviceCalibration.devicesIntrinsics

    deviceManager.disable_all_devices()
    resolutionWidth = 848
    resolutionHeight = 480
    frameRate = 90
    rsConfig.disable_stream(rs.stream.depth)
    rsConfig.disable_stream(rs.stream.color)
    rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
    rsConfig.enable_stream(rs.stream.infrared, 1, resolutionWidth, resolutionHeight, rs.format.y8, frameRate)
    #rsConfig.disable_stream(rs.stream.color)
    deviceManager.load_settings_json('DisparityShift5.json')
    deviceManager.enable_all_devices(enable_ir_emitter=True)
    #rsConfig.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)
    input("Calibration complete, press Enter to continue...")
    #time.sleep(6)
    fname = 1
    while True:
        data = AlignedData(transformation, intrinsics, deviceManager,fname)
        input("Data Collection complete, press Enter to continue...")
        time.sleep(6)
        fname+=1



    