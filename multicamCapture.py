import pyrealsense2 as rs 
import numpy as np 
import cv2 
from multicamManager import Calibrate
from helperfunctions.realsense_device_manager import DeviceManager
import time
from helperfunctions.realsense_device_manager import post_process_depth_frame
import copy
import threading
import sys
import multiprocessing 
import pickle
import queue

from pclpy import pcl
import pcl
import pcl.pcl_visualization


class Capture():
    def __init__(self,deviceManager, transformation):
        self.transformation = transformation
        self.deviceManager = deviceManager
        self.frames = {}
        self.processedFrames = {}
        #t1 = threading.Thread(target=self.capture())
        #t2 = threading.Thread(target=self.process())
        #1.start()
        #print('start')
    
    def capture(self):
        i=0
        file = open('dataStore', 'wb')
        while i<1000:
            try:
                savedData = {}
                timeStamp = str(time.time())
                timeFrame = self.deviceManager.poll_frames()
                for device,frames in timeFrame.items():
                    deviceData = {}
                    for frame in frames:
                        frameData = np.asanyarray(timeFrame[device][frame].get_data())
                        deviceData[frame]=copy.deepcopy(frameData)
                    savedData[device] = deviceData
                    
                #timeFrame['822512060553'][rs.stream.depth] = post_process_depth_frame(timeFrame['822512060553'][rs.stream.depth],temporal_smooth_alpha=0.1,temporal_smooth_delta=80)
                #timeFrame['823112060874'][rs.stream.depth] = post_process_depth_frame(timeFrame['823112060874'][rs.stream.depth],temporal_smooth_alpha=0.1,temporal_smooth_delta=80)
 
                #stframes[timeStamp] = copy.deepcopy(savedData)
                #self.frames[timeStamp] = savedData
                pickle.dump(copy.deepcopy(savedData),file)
                i+=1
                print(timeFrame['822512060853'][rs.stream.depth].get_frame_number())
                #print(len(frames.items()))

            except:
                pass
        file.close()

    def process(self):
        print('started')
        while True:
            try:
                for key,frame in self.frames.items():
                    value = self.frames.pop(key)
                    print(value)
            except:
                pass

    def loadData(self,filename):
        file = open(filename, 'rb')
        dataRead = []
        while 1:
            try:
                dataRead.append(pickle.load(file)) #this loads each frame in one at a time, this is where we can process each frame with the transformation matrix
            except EOFError:
                break


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
    #time.sleep(2)
    #t2.start()
    print('yay')
    #cap.capture()

    def captureThread(q,deviceManager):
        i=0
        alignTo = rs.stream.color
        align = rs.align(alignTo)
        while i<10000:
            savedData={}
            timeStamp = str(time.time())
            timeFrame = deviceManager.poll_frames(raw=True)
            devices={}
            for device,frames in timeFrame.items():
                #devices={}
                deviceData = {}
                alignedFrames = align.process(frames)
                deviceData[rs.stream.depth] = copy.deepcopy(np.asanyarray(alignedFrames.get_depth_frame().get_data()))
                deviceData[rs.stream.color] = copy.deepcopy(np.asanyarray(alignedFrames.get_color_frame().get_data()))
                #for frame in alignedFrames:
                #    frameData = np.asanyarray(frame.get_data())
                #    deviceData[frame]=copy.deepcopy(frameData)
                devices[device] = copy.deepcopy(deviceData)
            
            savedData[timeStamp]=devices
            q.put(copy.deepcopy(savedData))
            i+=1
            print(timeFrame['822512060853'][rs.stream.depth].get_frame_number())


    def processThread(q):
        file = open('dataStore','wb')

        while not q.empty():
            item = q.get()
            pickle.dump(copy.deepcopy(item),file)
            time.sleep(1/300)
            print('processed')
            q.task_done()
        print('queue finished')
        file.close()

    q = queue.Queue(maxsize=0)
    worker1 = threading.Thread(target=captureThread,args=(q,deviceManager))
    worker2 = threading.Thread(target=processThread,args=(q,))
    
    worker1.start()
    worker2.start()
    q.join()
    #cap.process()
    #cap.capture()
