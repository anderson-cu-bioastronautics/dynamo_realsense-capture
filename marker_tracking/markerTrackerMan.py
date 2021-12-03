from scipy.spatial import distance as dist
import numpy as np 
import cv2
import pickle
import os
import copy
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import ezc3d
from skimage import exposure


def dist3d(p1, p2):
    return sqrt((p1[0]-p2[0])^2 +(p1[1]-p2[1])^2+(p1[2]-p2[2])^2)

def get_surrDepths(contourPoints, depthFrame):
    depthVals = []
    if len(contourPoints)==2:
        depthVals.append(depthFrame[contourPoints[1], contourPoints[0]])
    else:
        for point in contourPoints:
            depthVals.append(depthFrame[point[1], point[0]])
    
    depth = np.mean(depthVals)
    return depth

def get_centroid(x, y, w, h):
    x1 = int(w / 2)
    y1 = int(h / 2)

    cx = x + x1
    cy = y + y1

    return (cx, cy)

def f2d23d(u,v,depthFrame, cameraIntrinsics, poseMat):


    ### The following lines of code are taken from the librealsense box_dimensioner_multicam example                                     ###
    ### See LICENSE.librealsense in root directory                                                                                                    ###
    ### https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/box_dimensioner_multicam/helper_functions.py ###
    
    #depth = get_surrDepths(contourPoints, depthFrame)
    depth = depthFrame[v,u]
    #print(depth)
    #cv2.imshow('depth',cv2.normalize(depthFrame, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F))
    #cv2.waitKey(1)

    
    x = (u - cameraIntrinsics['ppx'])/cameraIntrinsics['fx']
    y = (v - cameraIntrinsics['ppy'])/cameraIntrinsics['fy']
    z = (depth+4)/1000
    x = x*z
    y = y*z
    
    ###                                                                                                                                  ###


    points = np.asanyarray([x,y,z,1]) 

    pointsTransformed = np.matmul(poseMat, points)*1000 #muliply by 4x4 transformation matrix
    
    #pointsTransformed = np.true_divide(pointsTransformed[:3,:], pointsTransformed[[-1], :])
    
    return (pointsTransformed[0], pointsTransformed[1], pointsTransformed[2])
    

def rectContains(rect,pt): #https://stackoverflow.com/questions/33065834/how-to-detect-if-a-point-is-contained-within-a-bounding-rect-opecv-python
    logic = rect[0] < pt[0] < rect[0]+rect[2] and rect[1] < pt[1] < rect[1]+rect[3]
    return logic


def apply_brightness_contrast(input_img, brightness = 0, contrast = 0):
    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            highlight = 255
        else:
            shadow = 0
            highlight = 255 + brightness
        alpha_b = (highlight - shadow)/255
        gamma_b = shadow 
        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()
    if contrast != 0:
        f = 131*(contrast + 127)/(127*(131-contrast))
        alpha_c = f
        gamma_c = 127*(1-f)
        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)
    return buf

folderPath = "Z:/files/realsenseValidation/data/processing/001_BC_0.75" # r'D:/001_BB_0.75' python likes regular slash, not backslash
serial = '822512060853'
marker = 'LSAC'

saveDict = np.load(os.path.join(folderPath,serial+'.npy'), allow_pickle = True).item()
frameList=saveDict['frameList']
depthList=saveDict['depthList'] 
poseMat=saveDict['poseMat'] 
intrinsics=saveDict['intrinsics'] 
frames = {}
trackers = []
depths = {}
lastDepth = []
prevFail = []
jump = 0
prevImage = []
prevBox = []
px = 1
allPoints = np.empty((3, 1, len(frameList)))
f=0

for v,videoFrame in enumerate(frameList):
    frames[f] = [[],[],[]]
    depths[f] = []
    kernel = np.ones((19, 19), np.uint8)

    videoFrame = cv2.medianBlur(videoFrame,1)
    
    videoFrame = cv2.morphologyEx(videoFrame, cv2.MORPH_DILATE, np.ones((6,4),np.uint8))
    videoFrame2 = apply_brightness_contrast(videoFrame, brightness=50, contrast= 70)
    bvideoFrame = exposure.adjust_gamma(videoFrame2, 1)
    #bvideoFrame = apply_brightness_contrast(bvideoFrame, brightness=90, contrast= 0)

    #bvideoFrame = cv2.morphologyEx(bvideoFrame, cv2.MORPH_TOPHAT, np.ones((5,5),np.uint8))
    #markerFrameT = cv2.morphologyEx(videoFrame2, cv2.MORPH_GRADIENT, np.ones((5,5),np.uint8))
    #ret, markerFrame = cv2.threshold(markerFrameT,100,255,cv2.THRESH_BINARY)
    #markerFrame = cv2.morphologyEx(markerFrame, cv2.MORPH_DILATE, np.ones((2,2),np.uint8))
    
    #bvideoFrame = cv2.addWeighted(markerFrame, 0.8, apply_brightness_contrast(bvideoFrame, brightness=0, contrast=0),0.8,0.0)
    
    #bvideoFrame = cv2.morphologyEx(videoFrame2, cv2.MORPH_TOPHAT, np.ones((5,5),np.uint8))
    #clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(14,14))
    #bvideoFrame = clahe.apply(bvideoFrame)
    #bvideoFrame = apply_brightness_contrast(bvideoFrame, brightness=0, contrast=-10)
    #bvideoFrame = cv2.morphologyEx(videoFrame2, cv2.MORPH_GRADIENT, kernel)
    #videoFrame2 = cv2.bitwise_not(videoFrame2)
    #bvideoFrame = cv2.medianBlur(videoFrame, 1)
    #bvideoFrame = videoFrame
    fail=0
    if f <len(frameList):
        if trackers != []:
            depthFrame = depthList[f]
            print(depths[f-1])
            mask = np.asarray([(depthFrame<(lastDepth+70)) & (depthFrame>(lastDepth-70))]).astype(np.uint8)[0]
            #mask = mask.astype(np.uint8)
            #bvideoFrame = cv2.bitwise_and(bvideoFrame, bvideoFrame, mask = mask)
            (success, boxN) = trackers[0].update(bvideoFrame)
            if success:
                (x, y, w, h) = [int(v) for v in boxN]
                (x1, y1) = get_centroid(x,y,w,h)
                dist = np.sqrt((startX-x1)**2 + (startY-y1)**2)
                #if jump != []:
                #    diff = abs(dist-np.mean(jump))
                #else:
                #    diff = 1
                brightVal = bvideoFrame[y1,x1]
                #print(diff)
                #if prevImage:
                #(prevSuccess, prevboxN) = trackerTest.update(prevImage)
                #if prevSuccess:
                    #if rectContains(prevboxN, prevCentroid):
                if brightVal >= 0: #and diff < 20:
                    #jump.append(diff)
                    if prevImage:
                        (prevSuccess, prevBoxN) = trackers[0].update(prevImage[0])
                        (px, py, pw, ph) = [int(v) for v in prevBoxN]
                        (px1, py1) = get_centroid(px,py,pw,ph)
                        cv2.rectangle(prevImage[0], (px, py), (px + pw, py + ph), (155, 155, 155), 2)
                        cv2.imshow(serial+'prev', prevImage[0])
                        cv2.waitKey(1) & 0xFF
                        #time.sleep(1/20)
                        print(px1-px)
                        pbox = prevBox[0]
                        #pboxa = (pbox[0], pbox[1], 16,16)
                        if rectContains(pbox, (px1,py1)):
                    
                            cv2.rectangle(bvideoFrame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                            prevCentroid = (x1,y1)
                            frames[f] = f2d23d(x1,y1,depthList[f],intrinsics, poseMat)
                            toArray = np.empty_like(allPoints[:,:,f])
                            toArray[0] = frames[f][0]
                            toArray[1] = frames[f][1]
                            toArray[2] = frames[f][2]
                            allPoints[:,:, f] = toArray
                            currentDepth = depthList[f][y1,x1]
                            depths[f] = currentDepth
                            lastDepth = currentDepth
                            startX = x1
                            startY = y1
                            px = x
                            prevFail = []
                            prevImage = [bvideoFrame]
                            di = 8
                            prevBox = [(x1-di/2, y1-di/2, di, di)]
                            jump+=1
                        else:
                            fail = 1
                    #else:
                        #fail = 1
                else:
                    fail = 1
                #else: 
                    #fail = 1
            else:
                fail = 1
            if fail:
                trackers = []    
                depths[f] = []
            
        
                '''
                ret, videoFrameT = cv2.threshold(videoFrame,100,255,cv2.THRESH_BINARY)
                contours, hierarchy = cv2.findContours(videoFrameT,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                contourBoxes = [cv2.boundingRect(cnt) for cnt in contours]
                centroids = [get_centroid(cnt[0],cnt[1],cnt[2],cnt[3]) for cnt in contourBoxes]
                selectContains = [rectContains((x,y,w,h), (box[0],box[1])) for box in centroids]
                try:
                    whichContour = [i for i,s in enumerate(selectContains) if s][0]
                    contourPoints = np.vstack(contours[whichContour]).squeeze()
                    box = contourBoxes[whichContour]
                    (x, y, w, h) = [int(v) for v in boxN]
                    currentDepth = get_surrDepths(contourPoints, depthList[f])
                    cv2.rectangle(videoFrame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    (x1,y1) = get_centroid(x,y,w,h)
                    frames[f] = f2d23d(contourPoints,x1,y1,depthList[f],intrinsics, poseMat)
                    toArray = np.empty_like(allPoints[:,:,f])
                    toArray[0] = frames[f][0]
                    toArray[1] = frames[f][1]
                    toArray[2] = frames[f][2]
                    allPoints[:,:, f] = toArray
                    depths[f] = currentDepth
                    lastDepth = currentDepth
                
                    #if currentDepth - depths[f-1] > 10: #or dist3d(frames[f], frames[f-1]) > 20:
                    #    trackers = []
                    #    depths[f] = []
                    #    pass
                except:
                    pass
                '''
        if trackers == []:
            box = cv2.selectROI(serial, bvideoFrame, fromCenter=True, showCrosshair=True)
            if not box == (0,0,0,0):
                (x, y, w, h) = [int(v) for v in box]
                ret, videoFrameT = cv2.threshold(videoFrame2,200,255,cv2.THRESH_BINARY)
                contours, hierarchy = cv2.findContours(videoFrameT,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                contourBoxes = [cv2.boundingRect(cnt) for cnt in contours]
                centroids = [get_centroid(cnt[0],cnt[1],cnt[2],cnt[3]) for cnt in contourBoxes]
                #contourBoxes = [(box[0],box[1],box[2]+3,box[3]+3) for box in contourBoxes]
                selectContains = [rectContains((x,y,w,h), (box[0],box[1])) for box in centroids]
                try:
                    whichContour = [i for i,s in enumerate(selectContains) if s][0]
                    contourPoints = np.vstack(contours[whichContour]).squeeze()
                    centroid = centroids[whichContour]
                    w = contourBoxes[whichContour][2]+25
                    h = contourBoxes[whichContour][3]+20
                    box = (centroid[0]-w/2, centroid[1]-h/2, w, h)
                    (x, y, w, h) = [int(v) for v in box]
                    cv2.rectangle(bvideoFrame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    #print(contourContains, contourBoxes, (x,y))
                    tracker = cv2.TrackerCSRT_create()
                    tracker.init(bvideoFrame, box)
                    trackers.append(tracker)
                    x1,y1 = get_centroid(x,y,w,h)
                    prevCentroid = (x1,y1)
                    frames[f] = f2d23d(x1,y1,depthList[f],intrinsics, poseMat)
                    toArray = np.empty_like(allPoints[:,:,f])
                    toArray[0] = frames[f][0]
                    toArray[1] = frames[f][1]
                    toArray[2] = frames[f][2]
                    allPoints[:,:, f] = toArray
                    currentDepth = depthList[f][y1,x1]
                    depths[f] = currentDepth
                    lastDepth = currentDepth
                    startX = x1
                    startY = y1
                    prevImage = [bvideoFrame]
                    px = x1
                    prevBox = [(x1-16/2,y1-16/2,16,16)]
                except:
                    pass
            else:
                frames[f] = [[],[],[]] 

        cv2.imshow(serial, bvideoFrame)
        #cv2.imshow('blur',bvideoFrame)
        print(frames[f], f)
        #prevImage = bvideoFrame
        cv2.waitKey(1) & 0xFF
        #time.sleep(1/30)
        f+=1

c3d = ezc3d.c3d()

c3dtocopy = ezc3d.c3d("2019_05_07_01.c3d")

c3d['parameters']['POINT'] = c3dtocopy['parameters']['POINT']

c3d['parameters']['POINT']['RATE']['value'] = [90]
c3d['parameters']['POINT']['LABELS']['value'] = tuple([marker])
c3d['data']['points'] = allPoints
#c3d.add_parameter('POINT', 'DATA_START', 13)
c3d.add_parameter('POINT', 'FRAMES', len(frameList))
c3d.add_parameter('POINT', 'SCALE', -0.01)
#c3d.add_parameter('POINT', 'USED', len(lm_list))
c3d.write(os.path.join(folderPath, serial+marker+".c3d"))

#for f in list(frames.keys()):
    #ax.scatter(frames[f][0], frames[f][1], frames[f][2])
    #plt.draw()
    #plt.pause(1/15)
    #ax.cla()
