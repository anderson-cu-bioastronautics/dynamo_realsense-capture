##########################################################################################################################################
##                                               License: Apache 2.0. See LICENSE  file in root directory.	         	                ##
##########################################################################################################################################

from dynamo.realsense_device_manager import DeviceManager
from dynamo import calibration, stream
import pyrealsense2 as rs 
import os
import pptk
import numpy as np

cameraFiles = os.listdir('dynamo/sample_files')

config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
deviceManager = DeviceManager(rs.context(), config)

for camera in cameraFiles:
    deviceManager.enable_device_from_file('dynamo/sample_files/'+camera)


chessboardHeight = 4
chessboardWidth = 5
sideLength = 0.0762 #in meters

transformationMatrices = calibration.new('test.cal', deviceManager, chessboardWidth, chessboardHeight, sideLength)

allPoints = stream.single(deviceManager,transformationMatrices)

pptk.viewer(allPoints[:,0:3], allPoints[:,3:6])
