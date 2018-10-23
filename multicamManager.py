import pyrealsense2 as rs
import cv2
import numpy as np 
import time
from helperfunctions.realsense_device_manager import DeviceManager
import helperfunctions.calculate_rmsd_kabsch as rmsd
from plyfile import PlyData, PlyElement

class Calibrate():

    def __init__(self, device_manager):
        self.deviceManager = device_manager  
        #for frame in range(30): #disposes 30 frames to stabilize autoexposure
        time.sleep(5) #allow for auto-exposure to stabilize and frames to start coming in 
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
        #return self.devicesTransformation


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
                            points3D[0, index] = (corner[0]-depthIntrinsics.ppx)/depthIntrinsics.fx*depth
                            points3D[1, index] = (corner[1]-depthIntrinsics.ppy)/depthIntrinsics.fy*depth
                            points3D[2, index] = depth
                    self.devicesChessboardLocations[device] = corners, points2D, points3D, validPoints
                    chessboardDeviceCount += 1
                if not chessboardFound:
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
                #translationVector = rmsd.centroid(observedchessboardCentered) - np.matmul(rmsd.centroid(chessboardPointsCentered), rotationMatrix)
                translationVector = rmsd.centroid(validobservedchessboardPoints) - np.matmul(rmsd.centroid(validchessboardPoints), rotationMatrix)
                #rotationMatrix = rotationMatrix.transpose()
                poseMat = np.zeros((4,4))
                poseMat[:3,:3] = rotationMatrix
                poseMat[:3,3] = translationVector.flatten()
                poseMat[3,3] = 1
                self.devicesTransformation[serial] = [poseMat, rotationMatrix.transpose(), translationVector.transpose(), rmsdValue]

    
class AlignedData():
    def __init__(self, devicesTransformation, devicesIntrisics, deviceManager):
        self.devicesTransformation = devicesTransformation
        self.deviceManager = deviceManager
        self.devicesIntrinsics = devicesIntrisics
        self.stream()
        

    def depthFrametoPC(self, depthFrame, cameraIntrinsics, poseMat, rotationMatrix, translationVector):
        [height, width] = [depthFrame.get_height(), depthFrame.get_width()]
        nx = np.linspace(0, width-1, width)
        ny = np.linspace(0, height-1, height)
        u, v = np.meshgrid(nx, ny)
        x = (u.flatten() - cameraIntrinsics.ppx)/cameraIntrinsics.fx
        y = (v.flatten() - cameraIntrinsics.ppy)/cameraIntrinsics.fy
        z = np.asanyarray(depthFrame.get_data()).flatten() / 1000
        x = np.multiply(x,z)
        y = np.multiply(y,z)

        x = x[np.nonzero(z)]
        y = y[np.nonzero(z)]
        z = z[np.nonzero(z)]

        points = np.asanyarray([x,y,z])

        rot = np.transpose(rotationMatrix)
        trans = -np.matmul(np.transpose(rotationMatrix), translationVector)
        poseMat[:3,:3] = rot
        poseMat[:3,3] = trans
        poseMat[3,3] = 1

        n = points.shape[1] 
        points_ = np.vstack((points, np.ones((1,n))))
        points_trans_ = np.matmul(poseMat, points_)
        points_transformed = np.true_divide(points_trans_[:3,:], points_trans_[[-1], :])
        return points_transformed.T

    
    def stream(self):
        for (serial, [poseMat, rotationMatrix, translationVector, rmsdValue]) in self.devicesTransformation.items():
            cameraIntrinsics = self.devicesIntrinsics[serial][rs.stream.depth]
            frames = self.deviceManager.poll_frames()[serial]
            depthFrame = frames[rs.stream.depth]
            colorFrame = frames[rs.stream.color]

            points = self.depthFrametoPC(depthFrame, self.devicesIntrinsics[serial][rs.stream.depth], poseMat, rotationMatrix, translationVector)
            #print("wait")
            dt = [("x", 'f4'), ("y", 'f4'), ("z", 'f4')]
            verts = np.array(list(zip(*points.T)), dtype=dt)
            el = PlyElement.describe(verts, 'vertex')
            PlyData([el]).write(serial+"_DP2PC.ply")

            
            #Using RS export to PLY
            pc = rs.pointcloud()
            pc.map_to(colorFrame)
            points = rs.points()
            points = pc.calculate(depthFrame)
            vertices = np.array(np.asanyarray(points.get_vertices()).tolist())
            points.export_to_ply(serial+"_RSPC.ply",colorFrame)

            





if __name__=="__main__":
    rsConfig = rs.config()
    resolutionWidth = 848
    resolutionHeight = 480
    frameRate = 90
    #rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
    #rsConfig.enable_stream(rs.stream.infrared, 1, resolutionWidth, resolutionHeight, rs.format.rgb8, frameRate)
    #rsConfig.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)

    deviceManager = DeviceManager(rs.context(), rsConfig)
    deviceManager.enable_all_devices(enable_ir_emitter=True)
    deviceCalibration = Calibrate(deviceManager)
    transformation = deviceCalibration.devicesTransformation
    intrinsics = deviceCalibration.devicesIntrinsics
    input("Calibration complete, press Enter to continue...")
    data = AlignedData(transformation, intrinsics, deviceManager)




    