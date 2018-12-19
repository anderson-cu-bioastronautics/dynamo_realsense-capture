import pickle
from pclpy import pcl
import pcl
import pcl.pcl_visualization
import time

filename = 'dataStore'
file = open(filename, 'rb')
#dataRead = []
visual = pcl.pcl_visualization.CloudViewing()
time.sleep(2)
while 1:
    try:
        frame = pickle.load(file) #this loads each frame in one at a time, this is where we can process each frame with the transformation matrix
        visual.ShowColorACloud(frame)
    except EOFError:
        break
#print(dataRead)