import os
import pickle
import numpy as np

folderPath = r'S:/files/realsenseValidation/aboppana/pilot/001_BB_0.75'
savePath = r'D:/001_BB_0.75'

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

        with open(os.path.join(folderPath, file), 'rb') as fi:
            deviceData = pickle.load(fi)
        for camera in list(deviceData.keys()):
            if camera == serial:
                frameList.append(deviceData[camera]['infrared'])
                depthList.append(deviceData[camera]['depth'])
                poseMat = deviceData[camera]['poseMat']
                intrinsics = deviceData[camera]['intrinsics']
        f+=1
        print(str(f))

    saveDict = {}
    saveDict['frameList'] = frameList
    saveDict['depthList'] = depthList
    saveDict['poseMat'] = poseMat
    saveDict['intrinsics'] = intrinsics
    np.save(os.path.join(savePath, serial+'.npy'), saveDict, allow_pickle=True)