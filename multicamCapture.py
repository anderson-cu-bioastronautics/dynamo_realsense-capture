import pyrealsense2 as rs 
import numpy as np 
import cv2 
from multicamManager import Calibrate
from helperfunctions.realsense_device_manager import DeviceManager
import time

class Capture():
    def __init__(self,deviceManager, transformation):
        self.transformation = transformation
        self.deviceManager = deviceManager
        self.frames = {}
        self.processedFrames = {}

    def capture(self):
        while True:
            try:
                timeStamp = str(time.time())
                #for (serial, [poseMat, rmsdValue]) in self.transformation.items(): 
                timeFrame = self.deviceManager.poll_frames()
                self.frames[timeStamp] = timeFrame
                print(timeFrame['822512060553'][rs.stream.depth].get_frame_number())
            except:
                pass

if __name__=="__main__":
    rsConfig = rs.config()
    resolutionWidth = 848
    resolutionHeight = 480
    frameRate = 60
    rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
    rsConfig.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)
    #rsConfig.enable_stream(rs.stream.infrared, 2, resolutionWidth, resolutionHeight, rs.format.y8, frameRate)

    deviceManager = DeviceManager(rs.context(), rsConfig)
    deviceManager.enable_all_devices(enable_ir_emitter=True)
    i = 0
    """
    while True:
        try:
            frames = deviceManager.poll_frames()
            print(frames['822512060553'][rs.stream.depth].get_frame_number())
        except:
            continue
        #print(i)
        i+=1
    """
    cap = Capture(deviceManager, [])
    cap.capture()
