import pyrealsense2 as rs 
import numpy as np 
import cv2 
from multicamManager import Calibrate
from helperfunctions.realsense_device_manager import DeviceManager
import time
from helperfunctions.realsense_device_manager import post_process_depth_frame
import copy

class Capture():
    def __init__(self,deviceManager, transformation):
        self.transformation = transformation
        self.deviceManager = deviceManager
        self.frames = {}
        self.processedFrames = {}

    def capture(self):
        #filter = rs.spatial_filter()
        i=1
        while True:
            try:
                savedData = {}
                timeStamp = str(time.time())
                timeFrame = self.deviceManager.poll_frames()
                for device,frames in timeFrame.items():
                    deviceData = {}
                    for frame in frames:
                        frameData = np.asanyarray(timeFrame[device][frame].get_data())
                        deviceData[frame]=frameData
                    savedData[device] = deviceData
                    
                #timeFrame['822512060553'][rs.stream.depth] = post_process_depth_frame(timeFrame['822512060553'][rs.stream.depth],temporal_smooth_alpha=0.1,temporal_smooth_delta=80)
                #timeFrame['823112060874'][rs.stream.depth] = post_process_depth_frame(timeFrame['823112060874'][rs.stream.depth],temporal_smooth_alpha=0.1,temporal_smooth_delta=80)
 
                self.frames[timeStamp] = copy.deepcopy(savedData)

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
    """
    i = 0
    data = {}
    while True:
        try:
            timestamp = time.time()
            frames = deviceManager.poll_frames()
            print(frames['822512060553'][rs.stream.depth].get_frame_number())
            frames['822512060553'].keep()
            data[timestamp]=frames
        except:
            continue
        #print(i)
        i+=1
    """
    cap = Capture(deviceManager, [])
    cap.capture()
