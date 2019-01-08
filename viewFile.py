import pickle
from pclpy import pcl
import pcl
import pcl.pcl_visualization
import time
import numpy as np

def depthFrametoPC(depth, rgb, cameraIntrinsics, poseMat):
    starttime = time.time()
    [height,width] = np.array(depth.shape)
    #[height, width] = [depthFrame.get_height(), depthFrame.get_width()]
    #depth = np.asanyarray(depthFrame.get_data())
    #rgb = np.asanyarray(colorFrame.get_data())
    ##time1 = time.time()
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


filename = 'dataStore1'
file = open(filename, 'rb')
#dataRead = []
cloud = pcl.PointCloud_PointXYZRGBA()
visual = pcl.pcl_visualization.CloudViewing()
time.sleep(2)
i = 1
while 1:
    try:
        frame = pickle.load(file) #this loads each frame in one at a time, this is where we can process each frame with the transformation matrix
        cloud = pcl.PointCloud_PointXYZRGBA()
        allPoints = np.empty((0,4))
        #for (serial, [poseMat, rmsdValue]) in self.devicesTransformation.items():
        for camera,deviceData in frame.items():
            depthFrame = deviceData['depth']
            colorFrame = deviceData['color']
            cameraIntrinsics = deviceData['intrinsics']
            poseMat = deviceData['poseMat']
            points = depthFrametoPC(depthFrame, colorFrame, cameraIntrinsics, poseMat)
            allPoints = np.append(allPoints, points,axis=0)
        cloud.from_array(allPoints.astype('float32'))
        visual.ShowColorACloud(cloud)
        print(i)
        i+=1
    except EOFError:
        break
#print(dataRead)