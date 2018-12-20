import pickle
from pclpy import pcl
import pcl
import pcl.pcl_visualization
import time
import numpy as np

filename = 'dataStore5alliestatic'
file = open(filename, 'rb')
#dataRead = []
cloud = pcl.PointCloud_PointXYZRGBA()
visual = pcl.pcl_visualization.CloudViewing()
time.sleep(2)
i = 1
while 1:
    try:
        frame = pickle.load(file) #this loads each frame in one at a time, this is where we can process each frame with the transformation matrix
        allPoints = np.empty((0,4))
        for serial,points in frame.items():
            allPoints = np.append(allPoints, points,axis=0)
        cloud.from_array(allPoints.astype('float32'))
        visual.ShowColorACloud(cloud)
        print(i)
        i+=1
    except EOFError:
        break
#print(dataRead)