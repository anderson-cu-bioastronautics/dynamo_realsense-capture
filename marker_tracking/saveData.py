import os
import pickle
import numpy as np

folderPath = 'S:\\files\\realsenseValidation\\data\\pilot\\001_BB_0.75' #where the pickle files are located for a trial; these files are what is saved from the cameras
savePath = 'S:\\files\\realsenseValidation\\data\\processing\\001_BB_0.75' #where we want to put the npy files converted from the pickle files, so we can track markers on individual cameras

cameraList=['823112060874',
            '822512060522',
            '822512061105',
            '822512060553',
            '822512060853',
            '823112060112']


for serial in cameraList:
    frameFiles = sorted(os.listdir(folderPath))
    frameList = []
    depthList = []
    f=0
    for file in frameFiles:

        with open(os.path.join(folderPath, file), 'rb') as fi: #opens the file
            deviceData = pickle.load(fi) #dictionary of the 6 cameras
        for camera in list(deviceData.keys()): #key is camera serial number
            if camera == serial: #what is this?
                frameList.append(deviceData[camera]['infrared'])
                depthList.append(deviceData[camera]['depth'])
                poseMat = deviceData[camera]['poseMat']
                intrinsics = deviceData[camera]['intrinsics']
        f+=1
        print(str(f))

    saveDict = {}
    saveDict['frameList'] = frameList
    saveDict['depthList'] = depthList
    saveDict['poseMat'] = poseMat #4x4 transformation matrix from calibration
    saveDict['intrinsics'] = intrinsics #where points exist in 3d
    np.save(os.path.join(savePath, serial+'.npy'), saveDict, allow_pickle=True)