##########################################################################################################################################
##                             License: Apache 2.0. See LICENSE and LICENSE.librealsense files in root directory.		                ##
##########################################################################################################################################
## This code was inspired from the librealsense box_dimensioner_multicam example, and may contain certain lines of code from this file: ##
## (https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/box_dimensioner_multicam/calibration_kabsch.py).##                                          
##########################################################################################################################################



import pyrealsense2 as rs 
import cv2
import numpy as np
import time
import pickle

from .realsense_device_manager import DeviceManager
from .calculate_rmsd import *





def load(fileName):
    """ 
    Calibration parameters for previously connected cameras are loaded from a pickle file format.
    Calibration parameters for each camera include a 4x4 transformation matrix and rmsd error of calibration 

    Parameters
    ----------
    fileName : str
        Filename of stored calibration parameters.

    Returns
    -------
    devicesTransformation : dict
        Keys of camera's serial number holding dictionary of calibration parameters per camera

    Example
    -----
    load('savedCalibration.cal')
    """
    file = open(fileName,'rb')
    devicesTransformation = pickle.load(file)


def new(fileName,deviceManager, chessboardHeight, chessboardWidth, chessboardSquareSize):
    """ 
    New calibration parameters for each connected camera and are created and saved in a pickle file format.
    Calibration parameters for each camera include a 4x4 transformation matrix and rmsd error of calibration 

    Parameters
    ----------
    fileName : str
        Filename to store calibration parameters.
    
    deviceManager : DeviceManager object
        realsense_device_manager object which manages connections to all cameras
    
    chessboardHeight : int
        Number of chessboard intersections defining height of target chessboard

    chessboardWidth : int
        Number of chessboard intersections defining width of target chessboard
    
    chessboardSquareSize : float
        Dimension of side of chessboard (m)

    Returns
    -------
    devicesTransformation : dict
        dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

    Example
    -----
        new('savedCalibration.cal')
    """

    deviceManager.enable_all_devices()
    file = open(fileName,'wb')             
    time.sleep(1) #let autoexposure on cameras stabilize over one second 
    chessboardLocations = detectChessboard(deviceManager, chessboardHeight, chessboardWidth, chessboardSquareSize) #return locations of chessboards from reference frame of each camera
    devicesTransformations = poseTransformation(chessboardLocations, chessboardHeight, chessboardWidth, chessboardSquareSize) #return dictionary of 
    pickle.dump(devicesTransformations, file)
    file.close()
    return devicesTransformations


def detectChessboard(deviceManager, chessboardHeight, chessboardWidth, chessboardSquareSize):
    """ 
    Chessboard locations are computed for each connected RealSense camera

    Parameters
    ----------
    deviceManager : DeviceManager object
        realsense_device_manager object which manages connections to all cameras
    
    chessboardHeight: int
        Number of chessboard intersections defining height of target chessboard

    chessboardWidth: int
        Number of chessboard intersections defining width of target chessboard
    
    chessboardSquareSize: float
        Dimension of side of chessboard (m)

    Returns
    -------
    chessboardLocations : dict
        dictionary with keys of camera's serial number holding detected corners, local 2D and 3D coordinates of detected corners, and their valid depth points

    Example
    -----
        detectChessboard(deviceManager, chessboardHeight, chessboardWidth, chessboardSquareSize)
    """
    chessboardDeviceCount = 0
    devicesChessboardLocations = {}

    while len(devicesChessboardLocations) < len(deviceManager._enabled_devices): #iterate through detecting chessboard until all available devices see chessboard
        cameraFrames = deviceManager.poll_frames()
        devicesIntrinsics = deviceManager.get_device_intrinsics(cameraFrames)
        
        for device, frames in cameraFrames.items(): #this will iterate through each device's serial number (which are used as keys in the frames object)
            if not device in devicesChessboardLocations: #if the camera has not already detected the chessboard

                align = rs.align(rs.stream.depth) #align the color sensor to the depth sensor using the factory extrinsics
                alignedFrames = align.process(frames)
                
                colorImage = np.asanyarray(alignedFrames.get_color_frame().get_data()) 
                bwImage = cv2.cvtColor(colorImage,cv2.COLOR_BGR2GRAY) #convert color image to B&W image to use in openCV functions

                depthFrame = alignedFrames.get_depth_frame()
                depthIntrinsics = devicesIntrinsics[device][rs.stream.depth]  #obtain the depth sensor intrinsic properties

                chessboardFound, corners = cv2.findChessboardCorners(bwImage, (chessboardWidth, chessboardHeight)) #use openCV function to detect chessboard corners

                if chessboardFound: #if the camera sees the chessboard
                    print(device," sees the chessboard!")
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #subpix criteria
                    points2D = cv2.cornerSubPix(bwImage, corners, (11,11), (-1,-1), criteria) #further refine corners of chessboard using sub pixeling

                    cv2.drawChessboardCorners(bwImage, (chessboardWidth, chessboardHeight), points2D, chessboardFound) #draw chessboard corners in white over b&w image for verification
                    cv2.imshow(device,bwImage) #show the detected chessboard corners on the image
                    cv2.waitKey(50)

                    points2D = np.transpose(corners, (2,0,1)) 
                    points3D = np.zeros((3, len(points2D[0]))) #preallocate array to turn 2D chessboard corner points from camera into 3D points 
                    validPoints = [False] * len(points2D[0])

                    for index in range(len(points2D[0])): #iterate over every corner to find 3D location of chessboard corner
                        corner = points2D[:,index].flatten()
                        depth = depthFrame.as_depth_frame().get_distance(round(corner[0]), round(corner[1])) #this gets the depth at the pixel for each corner
                        if depth != 0 and depth is not None: #if the corner point has a valid depth value from the depth sensor
                            validPoints[index] = True #sets points which have a depth value as valid

                            #formualtion for finding 3D location of 2d point:
                            points3D[0, index] = (corner[0]-depthIntrinsics.ppx)/depthIntrinsics.fx*depth 
                            points3D[1, index] = (corner[1]-depthIntrinsics.ppy)/depthIntrinsics.fy*depth
                            points3D[2, index] = depth

                    devicesChessboardLocations[device] = corners, points2D, points3D, validPoints #save array for each camera of detected corners, their 2D locations, their 3D locations, and if they have a valid depth value
                    chessboardDeviceCount += 1

                if not chessboardFound: #if the camera doesn't see the chessboard
                    #chessboardDeviceCount = 0
                    cv2.imshow(device,bwImage)
                    cv2.waitKey(50)
                    print(device," cannot detect the chessboard!")

        time.sleep(1)
        cv2.destroyAllWindows()
    return devicesChessboardLocations
        


def poseTransformation(chessboardLocations, chessboardHeight, chessboardWidth, chessboardSquareSize):
    """ 
    Transformation matrices and error are computed for each connected RealSense camera

    Parameters
    ----------
    chessboardLocations : dict
        dictionary with keys of camera's serial number holding detected corners, local 2D and 3D coordinates of detected corners, and their valid depth points

    chessboardHeight: int
        Number of chessboard intersections defining height of target chessboard

    chessboardWidth: int
        Number of chessboard intersections defining width of target chessboard
    
    chessboardSquareSize: float
        Dimension of side of chessboard (m)

    Returns
    -------
    devicesTransformation: dict
        dictionary with keys of camera's serial number with each camera's 4x4 transformation matrix and root-mean squared error

    Example
    -----
        poseTransformation(chessboardLocations, chessboardHeight, chessboardWidth, chessboardSquareSize)
    """
    devicesTransformation = {}
    for (serial, [corners, points2D, points3D, validPoints]) in chessboardLocations.items(): #for every camera which has detected the chessboard

        if len(points2D[0])<5: #check if there are at least 5 points to be able to compute transformation matrix
            print(serial, " does not have enough points to have a valid depth for calculating the transformatoin")

        else:
            chessboardPoints = np.zeros((chessboardWidth*chessboardHeight,3), np.float32) #container for 3d coordinates of chessboard corners in global coordinates of chessboard
            chessboardPoints[:,:2] = np.mgrid[0:chessboardWidth, 0:chessboardHeight].T.reshape(-1,2)
            chessboardPoints = chessboardPoints.transpose() * chessboardSquareSize
            validchessboardPoints = chessboardPoints[:,validPoints].transpose()
            validobservedchessboardPoints = points3D[:, validPoints].transpose() #take chessboard points which have been detected by depth sensor

            chessboardPointsCentered = validchessboardPoints - centroid(validchessboardPoints) #center global chessboard points so that reference frame is same for every camera
            observedchessboardCentered = validobservedchessboardPoints - centroid(validobservedchessboardPoints) 

            rotationMatrix = kabsch(chessboardPointsCentered, observedchessboardCentered) #calculate rotation between local coordiante system and global coordinate system
            rmsdValue = kabsch_rmsd(chessboardPointsCentered, observedchessboardCentered) #calculate error of rotation matrix

            translationVector = centroid(validobservedchessboardPoints) - np.matmul(centroid(validchessboardPoints), rotationMatrix) #calculate translation between local and global coordinate system
            trans = -np.matmul(rotationMatrix, translationVector.transpose())

            poseMat = np.zeros((4,4)) #build 4x4 transformation matrix
            poseMat[:3,:3] = rotationMatrix
            poseMat[:3,3] = trans.flatten()
            poseMat[3,3] = 1
            
            devicesTransformation[serial] = [poseMat, rmsdValue]
    return devicesTransformation

if __name__ == "__main__":
    new('newCalibration.cal')