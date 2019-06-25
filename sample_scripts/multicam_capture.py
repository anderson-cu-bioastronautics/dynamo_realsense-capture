##########################################################################################################################################
##                             License: Apache 2.0. See LICENSE file in root directory.		                                            ##
##########################################################################################################################################

import pyrealsense2 as rs
import cv2
import numpy as np 
import time

from dynamo.realsense_device_manager import DeviceManager
import dynamo.calibration as calibration
import dynamo.stream as stream

import copy
import threading
import sys
import multiprocessing 
import pickle
import queue
import os

import argparse



if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--load", help="load calibration",
                        nargs='?')
    parser.add_argument("--new", help="new calibration",
                        nargs='?',default="new.cal")

    parser.add_argument("--folder", help="data folder",
                        nargs = '?', default="data")
    parser.add_argument("--time", help="time to collect data (s)",
                        nargs = '?', default="10")
    args = parser.parse_args()

    rsConfig = rs.config()
    if args.load:
        print(os.path.join(os.getcwd(),args.load))
        file = open(os.path.join(os.getcwd(),args.load),'rb')
        transformation = pickle.load(file)
        file.close()
        #transformation = calibration.load(os.path.join(os.getcwd(),args.load))
        print(transformation)
        deviceManager = DeviceManager(rs.context(), rsConfig)
        deviceManager.enable_all_emitters()
    elif args.new:
        resolutionWidth = 848
        resolutionHeight = 480
        frameRate = 30
        rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
        rsConfig.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)

        deviceManager = DeviceManager(rs.context(), rsConfig)
        deviceManager.enable_all_emitters()
        deviceManager.load_settings_json('calibrationSettings.json')
        cameraOrder = [
                        '822512060522', 
                        '823112060874',
                        '823112060112',
                        '822512060553', 
                        '822512060853', 
                        '822512061105']
        transformation = calibration.newIterative(args.new,deviceManager, cameraOrder, 4,5,0.0762)
        deviceManager.disable_all_devices()
        rsConfig.disable_stream(rs.stream.depth)
        rsConfig.disable_stream(rs.stream.color)

    resolutionWidth = 848
    resolutionHeight = 480
    frameRate = 90

    rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
    rsConfig.enable_stream(rs.stream.infrared, 1, resolutionWidth, resolutionHeight, rs.format.y8, frameRate)
    
    deviceManager.load_settings_json('markerSettings.json')
    deviceManager.enable_all_devices()
    
    input("Calibration complete, press Enter to continue...")
    
    script_path = os.path.abspath(__file__)
    scriptDir = os.path.split(script_path)[0]

    if not os.path.isdir(os.path.join(os.getcwd(), args.folder)):
        os.mkdir(args.folder) #make base folder if it doesn't exist already
    iteration = 1
    while True:
        loc = args.folder+'\\'+str(format(iteration, '02d'))
        saveDirectory = os.path.join(os.getcwd(), loc)
        if not os.path.isdir(saveDirectory):
            os.mkdir(args.folder+'\\'+str(format(iteration, '02d'))) #make iteration folder if it doesn't exist already 
        data = stream.start(deviceManager, transformation, saveDirectory,args.time)
        input("Data Collection complete, press Enter to continue...")
        
        iteration+=1



    