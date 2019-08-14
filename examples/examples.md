# Examples

## Camera Setup and Streaming

To setup the cameras in python and receive a single depth frame, you must specify a `rs.config()` object, which defines the desired streams. 
A stream is enabled in `rs.config()` by defining its sensor, resolution, format, and frame rate. 
Available sensors are defined with `rs.stream` objects, including:
* rs.stream.depth
* rs.stream.infrared
* rs.stream.color

Since RealSense cameras contain two infrared sensors, the requested sensor must be identified with either a `1` or `0`, following the `rs.stream` argument. 
This argument is not required for depth or color streams. 

Each stream has its own corresponding formats in which to save data.
See librealsense **link** for more information. 
Recommended formats for each stream are: 
* rs.stream.depth
  * rs.format.z16
* rs.stream.infrared
  * rs.format.y8
* rs.stream.color
  * rs.format.bgr8

Certain frame rates and resolutions are not compatible with certain sensors. 
See librealsense **link** for more information. 

  
```python
from dynamo.realsense_device_manager import DeviceManager
import pyrealsense2 as rs

resolutionWidth = 848
resolutionHeight = 480
frameRate = 90

rsConfig = rs.config()
rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)

deviceManager = DeviceManager(rs.context(), rsConfig)

deviceManager.enable_all_devices()

frame = deviceManager.poll_frames()
```
## Loading a Virtual Camera

Alternatively, we can load in a virtual camera using a saved stream file in the `.bag` format. 


```python
from dynamo.realsense_device_manager import DeviceManager
import pyrealsense2 as rs

resolutionWidth = 848
resolutionHeight = 480
frameRate = 90

rsConfig = rs.config()
rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)

deviceManager = DeviceManager(rs.context(), rsConfig)

deviceManager.enable_device_from_file('path/to/file.bag')

frame = deviceManager.poll_frames()

```
## Camera Settings
A variety of camera settings are available to tune the camera's capture settings to your scence and environment. 
These settings are parsed in a ``.json`` file. 
The settings are best manipulated using the [Intel RealSense Viewer](https://software.intel.com/en-us/realsense/d400/get-started), and saved as a ``.json`` file. 
Sample capture settings are included in the [capture_settings](../capture_settings) folder. 

Capture settings can be loaded for all cameras. 

```python
from dynamo.realsense_device_manager import DeviceManager
import pyrealsense2 as rs

resolutionWidth = 848
resolutionHeight = 480
frameRate = 30

rsConfig = rs.config()
rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
rsConfig.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)

deviceManager = DeviceManager(rs.context(), rsConfig)

deviceManager.load_settings_json('calibrationSettings.json')

```


## Camera Calibration

To calibrate the cameras to a global coordinate system, you will first need to create a chessboard to be used as the calibration target.
The chessboard must have square sides. 
A sample chessboard template is provied **link to sample chessboard**



The chessboard must be placed such that all corners of the chessboard are seen by all connected cameras. 
The `calibration` function can be used to return the 4x4 transformation matrix for each connected camera which aligns it to the center of the chessboard. 
The transformation matrices of all connected cameras are saved as a pickle file with the specified filename.

```python
from dynamo.realsense_device_manager import DeviceManager
from dynamo import calibration
import pyrealsense2 as rs

resolutionWidth = 848
resolutionHeight = 480
frameRate = 30

rsConfig = rs.config()
rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
rsConfig.enable_stream(rs.stream.color, resolutionWidth, resolutionHeight, rs.format.bgr8, frameRate)

deviceManager = DeviceManager(rs.context(), rsConfig)

fileName = 'newCalibration.cal' #file to save transformation matrices
chessboardWidth = 4 #number of corners in width
chessboardHeight = 5 #number of corners in height
chessboardSquareSize = 0.0762 #chessboard square size in meters

transformationMatrices = calibration.new(fileName, deviceManager, chessboardWidth, chessboardHeight, chessboardSquareSize)
```
## Camera Streaming to Disk
Once the cameras have been calibrated, they can be streamed and saved to disk.
The function `stream.start()` takes as arguments the `Device Manager` object, the calibration transformation matrices, the folder in which to save the data, and the time to collect data. 

Previously saved calibration transformation matrices can be loaded using `calibration.load`.

```python
from dynamo.realsense_device_manager import DeviceManager
from dynamo import calibration, stream
import pyrealsense 2 as rs

resolutionWidth = 848
resolutionHeight = 480
frameRate = 90

rsConfig = rs.config()
rsConfig.enable_stream(rs.stream.depth, resolutionWidth, resolutionHeight, rs.format.z16, frameRate)
rsConfig.enable_stream(rs.stream.infrared, 1, resolutionWidth, resolutionHeight, rs.format.y8, frameRate)

transformationMatrices = calibration.load('newCalibration.cal')

deviceManager = DeviceManager(rs.context(), rsConfig)
deviceManager.enable_all_devices()

stream.start(deviceManager, transformationMatrices, 'saveFolder', time)

```

## Pointcloud Viewing from Saved Frames

To view saved frames as a pointcloud, one can use the `view` function. 
This function has the option to be run either from the command line, or as a function. 
By default, this function only displays every 10 frames to allow for quick review of the collected data, but can be specified to display every frame. 

```python
from dynamo import view

folder = 'testData'
iteration = '1'
full = 1 #set to 1 to view all frames, set to 0 to view every 10 frames

view.viewPointCloud(folder,iteration,full)
```

## Getting Marker Locations from Frame

The `marker` function provides the ability to get the location of any detected markers in the frame. 
The frame can be loaded from disk and passed to the `marker.detect()` function. 

```python
from dyamo import markers
import os
import pickle

folder = 'testData/1'
for frameFile in os.listdir(folder):
    with open(folder+'/'+frameFile, 'rb') as f:
        frame = pickle.load(f)

    markerList = markers.detect(frame)


```


