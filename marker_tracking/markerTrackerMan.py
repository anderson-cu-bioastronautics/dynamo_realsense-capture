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


def dist3d(p1, p2): #calculating distance from 2 points in 3D
    return sqrt((p1[0]-p2[0])^2 +(p1[1]-p2[1])^2+(p1[2]-p2[2])^2)#from p1 and p2 vectors

def get_surrDepths(contourPoints, depthFrame): 
    """ 
Parameters
------------
contourPoints : list
                list of points that we want the depth from
depthFrame    : 2D array
                2D image of depth instead of rgb for each pixel

Returns
---------
depth       :   float
                mean of all depth values at every contour point"""
    depthVals = []
    if len(contourPoints)==2:#makes sure python iterates through each column in contourPoints instead of row (if 1 contourPoint is given)
        depthVals.append(depthFrame[contourPoints[1], contourPoints[0]])
    else:
        for point in contourPoints:
            depthVals.append(depthFrame[point[1], point[0]])
    
    depth = np.mean(depthVals)
    return depth

def get_centroid(x, y, w, h): #x,y are top left, finds center of box given width w and height h
    x1 = int(w / 2)
    y1 = int(h / 2)

    cx = x + x1
    cy = y + y1

    return (cx, cy)

def f2d23d(u,v,depthFrame, cameraIntrinsics, poseMat):
    """
    Parameters
    --------
    u          :   int
                    x coordinate
    v          :   int
                    y coordinate
    depthFrame :    2D array
                    array of depth values instead of rgb
    cameraIntrinsics :  dict
                        contains internal calibration of depth sensor (values are ppx,ppy,fx,fy(focal lengths))
    poseMat         :   2D array 4x4
                        external calibration matrix, converts from 3d to camera points, is a transformation matrix
    """

    ### The following lines of code are taken from the librealsense box_dimensioner_multicam example                                     ###
    ### See LICENSE.librealsense in root directory                                                                                                    ###
    ### https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/box_dimensioner_multicam/helper_functions.py ###
    
    #depth = get_surrDepths(contourPoints, depthFrame)
    depth = depthFrame[v,u]#getting the depth value at given point (u,v)
    #print(depth)
    #cv2.imshow('depth',cv2.normalize(depthFrame, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F))
    #cv2.waitKey(1)

    
    x = (u - cameraIntrinsics['ppx'])/cameraIntrinsics['fx']
    y = (v - cameraIntrinsics['ppy'])/cameraIntrinsics['fy']
    z = (depth+4)/1000#is actually 's' in dynamo paper
    x = x*z
    y = y*z #because we know cameraIntrinsics, we can invert to find [x;y;z], 3D real world location
    
    ###                                                                                                                                  ###


    points = np.asanyarray([x,y,z,1]) 

    pointsTransformed = np.matmul(poseMat, points)*1000 #muliply by 4x4 transformation matrix; gets all cameras to use the same origin
    
    #pointsTransformed = np.true_divide(pointsTransformed[:3,:], pointsTransformed[[-1], :])
    
    return (pointsTransformed[0], pointsTransformed[1], pointsTransformed[2])
    

def rectContains(rect,pt): #https://stackoverflow.com/questions/33065834/how-to-detect-if-a-point-is-contained-within-a-bounding-rect-opecv-python
    logic = rect[0] < pt[0] < rect[0]+rect[2] and rect[1] < pt[1] < rect[1]+rect[3]
    return logic #this tells if the point is within the given box


def apply_brightness_contrast(input_img, brightness = 0, contrast = 0): #how does this work/what does this do? 
    """
    Parameters
    ------------
    input_img   :   2d array
                    grayscale pixel value (0-255)
    brightness  :   float
                    how bright the image is overall 
    contrast    :   float   
                    how much contrast
    
    https://stackoverflow.com/questions/39308030/how-do-i-increase-the-contrast-of-an-image-in-python-opencv
    See second answer (by bfris)
    """
    if brightness != 0: #only change brightness when we want it
        if brightness > 0: 
            shadow = brightness 
            highlight = 255 
        else: #brightness can be negative
            shadow = 0
            highlight = 255 + brightness 
        alpha_b = (highlight - shadow)/255#keeps contrast the same
        gamma_b = shadow #keeps contrast the same
        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()
    if contrast != 0:
        f = 131*(contrast + 127)/(127*(131-contrast))
        alpha_c = f
        gamma_c = 127*(1-f)
        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)
    return buf

folderPath = "S:/files/realsenseValidation/data/processing/001_BB_0.75"#where the .npy file is located for the camera and marker we want to track # r'D:/001_BB_0.75' python likes regular slash, not backslash, we're processing this trial
serial = '823112060112' #this camera specifically 
marker = 'LIAT' #this specific marker

saveDict = np.load(os.path.join(folderPath,serial+'.npy'), allow_pickle = True).item() #reading all the data and creating variables
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

for v,videoFrame in enumerate(frameList): #goes through every frame, v is index, videoFrame is the actual frame
    #enumerate steps through each object, gives index and the object value
    frames[f] = [[],[],[]]
    depths[f] = []
    kernel = np.ones((19, 19), np.uint8)
    #next 4 can be adjusted to improve tracking
    videoFrame = cv2.medianBlur(videoFrame,1)#blur the frame in IR to make marker more visible
    
    videoFrame = cv2.morphologyEx(videoFrame, cv2.MORPH_DILATE, np.ones((6,4),np.uint8))#makes the marker bigger almost like bolding it
    videoFrame2 = apply_brightness_contrast(videoFrame, brightness=80, contrast= 80)#adjust brightness and contrast
    bvideoFrame = exposure.adjust_gamma(videoFrame2, 1)#adjusts brightness/contrast
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
    fail=0#if it cannot find marker, fail=1. Initialize with 0
    if f <len(frameList): #Not the last frame
        if trackers != []:#as long as there is something within the tracking box
            depthFrame = depthList[f]
            print(depths[f-1])
            mask = np.asarray([(depthFrame<(lastDepth+70)) & (depthFrame>(lastDepth-70))]).astype(np.uint8)[0] #changes threshold on depth
            #if the value jumps from 10 to 80, person probably didn't move 7 cm in 1/90th of a second, could use different value
            #only looks within a certain depth, sets everything outside the threshold to zero
            #mask = mask.astype(np.uint8)
            #bvideoFrame = cv2.bitwise_and(bvideoFrame, bvideoFrame, mask = mask)
            (success, boxN) = trackers[0].update(bvideoFrame)#applies current tracker to next frame
            #bvideoFrame is final after processing
            if success:#tracker found the marker
                (x, y, w, h) = [int(v) for v in boxN]
                (x1, y1) = get_centroid(x,y,w,h)
                dist = np.sqrt((startX-x1)**2 + (startY-y1)**2)#startX/Y is location of previous frame tracker, shows how much it moved between frames
                #if jump != []:
                #    diff = abs(dist-np.mean(jump))
                #else:
                #    diff = 1
                brightVal = bvideoFrame[y1,x1]#grayscale value of center of tracker, y,x because coordinate system is weird in cv
                #print(diff)
                #if prevImage:
                #(prevSuccess, prevboxN) = trackerTest.update(prevImage)
                #if prevSuccess:
                    #if rectContains(prevboxN, prevCentroid):
                if brightVal >= 0: #and diff < 20: #if there is a marker in the box (indicated by brightness), can change brightVal as a catch function
                    #jump.append(diff)
                    if prevImage:#not the first frame
                        (prevSuccess, prevBoxN) = trackers[0].update(prevImage[0]) #run tracker on previous image
                        (px, py, pw, ph) = [int(v) for v in prevBoxN]
                        (px1, py1) = get_centroid(px,py,pw,ph)
                        cv2.rectangle(prevImage[0], (px, py), (px + pw, py + ph), (155, 155, 155), 2) #draws a rectangle
                        cv2.imshow(serial+'prev', prevImage[0])#rectangle over location of marker in previous frame
                        cv2.waitKey(1) & 0xFF
                        #time.sleep(1/20)
                        print(px1-px)
                        pbox = prevBox[0]
                        #pboxa = (pbox[0], pbox[1], 16,16)
                        if rectContains(pbox, (px1,py1)):#if previous tracked point is within the box itself
                    
                            cv2.rectangle(bvideoFrame, (x, y), (x + w, y + h), (255, 255, 255), 2)#draw rectangle on current frame with current tracking estimate
                            prevCentroid = (x1,y1)#set this up for the next frame
                            frames[f] = f2d23d(x1,y1,depthList[f],intrinsics, poseMat)#gives 3D location of markers
                            toArray = np.empty_like(allPoints[:,:,f])#creates an array for marker location
                            toArray[0] = frames[f][0]
                            toArray[1] = frames[f][1]
                            toArray[2] = frames[f][2]
                            allPoints[:,:, f] = toArray
                            currentDepth = depthList[f][y1,x1]#gets depth at current marker
                            depths[f] = currentDepth
                            lastDepth = currentDepth#reset for next frame through line 241
                            startX = x1
                            startY = y1
                            px = x
                            prevFail = []
                            prevImage = [bvideoFrame]
                            di = 8
                            prevBox = [(x1-di/2, y1-di/2, di, di)]
                            jump+=1
                        else:
                            fail = 1#catch function (maybe?)
                    #else:
                        #fail = 1
                else:
                    fail = 1#if no marker in the box, we fail it
                #else: 
                    #fail = 1
            else:
                fail = 1#if the tracker fails to find a marker
            if fail:
                trackers = []    #deletes trackers
                depths[f] = [] #makes depths empty at that frame (we didn't find a marker)
            
        
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
        if trackers == []: #if a tracker failed, we want to make a new one
            box = cv2.selectROI(serial, bvideoFrame, fromCenter=True, showCrosshair=True)#UI to create the selection box
            if not box == (0,0,0,0): #make sure we actaully select a box
                (x, y, w, h) = [int(v) for v in box]
                ret, videoFrameT = cv2.threshold(videoFrame2,200,255,cv2.THRESH_BINARY)#threshold for brightness
                contours, hierarchy = cv2.findContours(videoFrameT,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#looks for a blob
                contourBoxes = [cv2.boundingRect(cnt) for cnt in contours]#finds all blobs in defined box
                centroids = [get_centroid(cnt[0],cnt[1],cnt[2],cnt[3]) for cnt in contourBoxes]#gets centroid of box centered on marker
                #contourBoxes = [(box[0],box[1],box[2]+3,box[3]+3) for box in contourBoxes]
                selectContains = [rectContains((x,y,w,h), (box[0],box[1])) for box in centroids]#makes sure there is a marker in the box
                try: #create tracker, fine-tune box around the marker, setting things up for iterating 
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
                    pass#if it doesn't work (if we skip the frame because it's still not visible), just skip the frame
            else:
                frames[f] = [[],[],[]] 

        cv2.imshow(serial, bvideoFrame) #show the current (processed) frame
        #cv2.imshow('blur',bvideoFrame)
        print(frames[f], f)
        #prevImage = bvideoFrame
        cv2.waitKey(1) & 0xFF
        #time.sleep(1/30)
        f+=1

c3d = ezc3d.c3d() #loads up the c3d library, saves marker locations

c3dtocopy = ezc3d.c3d("2019_05_07_01.c3d")#uses this as a "baseline" file

c3d['parameters']['POINT'] = c3dtocopy['parameters']['POINT']#defines mocap marker

c3d['parameters']['POINT']['RATE']['value'] = [90]#defines framerate
c3d['parameters']['POINT']['LABELS']['value'] = tuple([marker])#defines marker name
c3d['data']['points'] = allPoints#array that stores our marker (3D location in space)
#c3d.add_parameter('POINT', 'DATA_START', 13)
c3d.add_parameter('POINT', 'FRAMES', len(frameList))#number of frames
c3d.add_parameter('POINT', 'SCALE', -0.01)#scale (mm)
#c3d.add_parameter('POINT', 'USED', len(lm_list))
c3d.write(os.path.join(folderPath, serial+marker+".c3d")) #saves into the indicated folder path

#for f in list(frames.keys()):
    #ax.scatter(frames[f][0], frames[f][1], frames[f][2])
    #plt.draw()
    #plt.pause(1/15)
    #ax.cla()
