import pickle
from pclpy import pcl
import pcl
import pcl.pcl_visualization
import time
import numpy as np
import multiprocessing
import threading
import queue
import cv2
def depthFrametoPC(deviceData):
    #starttime = time.time()
    depth = deviceData['depth']
    infrared = deviceData['infrared']
    rgb = cv2.cvtColor(np.asanyarray(infrared),cv2.COLOR_GRAY2RGB)
    cameraIntrinsics = deviceData['intrinsics']
    poseMat = deviceData['poseMat']
    [height,width] = np.array(depth.shape)
    nx = np.linspace(0, width-1, width)
    ny = np.linspace(0, height-1, height)
    u, v = np.meshgrid(nx, ny)
    x = (u.flatten() - cameraIntrinsics['ppx'])/cameraIntrinsics['fx']
    y = (v.flatten() - cameraIntrinsics['ppy'])/cameraIntrinsics['fy']
    z = depth.flatten() / 1000
    x = np.multiply(x,z)
    y = np.multiply(y,z)
    ##time2 = time.time()
    #x = x[np.nonzero(z)]
    #y = y[np.nonzero(z)]
    #z = z[np.nonzero(z)]

    rgbB = rgb[:,:,0].flatten().astype(int)
    rgbG = rgb[:,:,1].flatten().astype(int)
    rgbR = rgb[:,:,2].flatten().astype(int)
    rgbPC = rgbR<<16|rgbG<<8|rgbB
    ##time3=time.time()
    points = np.asanyarray([x,y,z])

    n = points.shape[1] 
    points_ = np.vstack((points, np.ones((1,n))))
    points_trans_ = np.matmul(poseMat, points_)
    points_transformed = np.true_divide(points_trans_[:3,:], points_trans_[[-1], :])
    ##time4=time.time()
    allPoints = np.asanyarray([points_transformed[0,:],points_transformed[1,:],points_transformed[2,:],rgbPC]).T
    ##times = np.array([time1,time2,time3,time4])-starttime
    ##print(times)
    return allPoints


def importThread(qFiles,fileName):
    while 1:
        try:
            frame = pickle.load(fileName)
            qFiles.put(frame)
        except EOFError:
            return

def processThread(qFiles):
    import pcl
    p = multiprocessing.Pool(6)
    visual = pcl.pcl_visualization.CloudViewing()
    i=0
    while not qFiles.empty():
        if i >5:
            cloud = pcl.PointCloud_PointXYZRGBA()
            frame = qFiles.get()
            listP = [data for serial, data in frame.items()]
            """
            for serial, data in frame.items():
                listP.append(data)
            """
            cameraPoints = p.map(depthFrametoPC, listP)
            allPoints = np.empty((0,4))
            for camera in cameraPoints:
                allPoints = np.append(allPoints, camera, axis=0)
            #p.close()
            cv2.imshow('color', frame['822512060522']['infrared'])
            cv2.waitKey(1)
            cloud.from_array(allPoints.astype('float32'))
            visual.ShowColorACloud(cloud)
            #qDisplay.put(allPoints)
            print(i)
        i+=1
        qFiles.task_done()

def main():
    filename = 'dataStore1.pickle'
    fileName = open(filename, 'rb')
    qFiles = queue.Queue(maxsize=0)
    worker1 = threading.Thread(target=importThread,args=(qFiles,fileName))
    worker2 = threading.Thread(target=processThread,args=(qFiles,))
    worker1.start()
    worker1.join()
    time.sleep(0.25)
    worker2.start()
    worker2.join()
    qFiles.join()


if __name__=="__main__":
    multiprocessing.freeze_support()
    main()

