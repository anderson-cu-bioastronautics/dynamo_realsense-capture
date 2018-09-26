import pyrealsense2 as rs
import cv2
import numpy as np 
import time
from helperfunctions.realsense_device_manager import DeviceManager
import helperfunctions.calculate_rmsd_kabsch as rmsd

class Calibrate():

    def __init__(self, device_manager):
        self.deviceManager = device_manager  
        #for frame in range(30): #disposes 30 frames to stabilize autoexposure
        time.sleep(5)
        frames = self.deviceManager.poll_frames()
        self.devicesIntrinsics = self.deviceManager.get_device_intrinsics(frames)
        self.devicesChessboardLocations = {}
        self.devicesTransformation = {}
        
        #Define Calibration Target
        self.chessboardHeight = 6 #squares
        self.chessboardWidth = 9 #squares
        self.chessboardSquareSize = 0.0253 #m
        
        #Perform Calibration steps
        self.detectChessboard()
        self.poseTransformation()


    def detectChessboard(self):
        chessboardDeviceCount = 0
        while chessboardDeviceCount < len(self.deviceManager._available_devices):
            cameraFrames = self.deviceManager.poll_frames()
            for device in cameraFrames.keys(): #this will iterate through each device's serial number (which are used as keys in the frames object)
                singleCameraFrame = cameraFrames[device]
                infraredImage = np.asanyarray(singleCameraFrame[rs.stream.infrared,1].get_data())
                depthFrame = singleCameraFrame[rs.stream.depth]
                depthIntrinsics = self.devicesIntrinsics[device][rs.stream.depth]                
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                chessboardFound, corners = cv2.findChessboardCorners(infraredImage, (self.chessboardWidth, self.chessboardHeight))
                if chessboardFound:
                    print(device," sees the chessboard!")
                    points2D = cv2.cornerSubPix(infraredImage, corners, (11,11), (-1,-1), criteria)
                    points2D = np.transpose(corners, (2,0,1))
                    points3D = np.zeros((3, len(points2D[0])))
                    validPoints = [False] * len(points2D[0])
                    for index in range(len(points2D[0])):
                        corner = points2D[:,index].flatten()
                        depth = depthFrame.as_depth_frame().get_distance(round(corner[0]), round(corner[1])) #this gets the depth at the pixel
                        if depth != 0 and depth is not None:
                            validPoints[index] = True #sets points which have a depth value as valid
                            points3D[0, index] = (corner[0]-depthIntrinsics.ppx)/(depthIntrinsics.fx*depth)
                            points3D[1, index] = (corner[1]-depthIntrinsics.ppy)/(depthIntrinsics.fy*depth)
                            points3D[2, index] = depth
                    self.devicesChessboardLocations[device] = corners, points2D, points3D, validPoints
                    chessboardDeviceCount += 1
                else:
                    print(device," cannot detect the chessboard!")

                   
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
                translationVector = rmsd.centroid(validobservedchessboardPoints) - np.matmul(rmsd.centroid(validchessboardPoints), rotationMatrix)
                
                poseMat = np.zeros((4,4))
                poseMat[:3,:3] = rotationMatrix
                poseMat[:3,3] = translationVector.flatten()
                poseMat[3,3] = 1
                self.devicesTransformation[serial] = [poseMat, rotationMatrix, translationVector, rmsdValue]

    def applyTransformation(self):



if __name__=="__main__":
    rs_config = rs.config()
    resolutionWidth = 1280
    resolutionHeight = 720
    frameRate = 15
    rs_config.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
    rs_config.enable_stream(rs.stream.infrared, 1, resolutionWidth, resolutionHeight, rs.format.y8, frameRate)
    rs_config.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)

    device_manager = DeviceManager(rs.context(), rs_config)
    device_manager.enable_all_devices()
    Calibrate(device_manager)




    