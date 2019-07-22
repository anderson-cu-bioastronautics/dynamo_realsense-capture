# dynamo

# dynamo.calibration

## load
```python
load(fileName)
```

Calibration parameters for previously connected cameras are loaded from a pickle file format.
Calibration parameters for each camera include a 4x4 transformation matrix and rmsd error of calibration

Parameters
----------
fileName : str
    Filename of stored calibration parameters.

Returns
-------
devicesTransformation : dict
    Keys of camera's serial number holding dictionary of calibration parameters per camera

Example
-----
load('savedCalibration.cal')

## new
```python
new(fileName, deviceManager, chessboardHeight, chessboardWidth, chessboardSquareSize)
```

New calibration parameters for each connected camera and are created and saved in a pickle file format.
Calibration parameters for each camera include a 4x4 transformation matrix and rmsd error of calibration

Parameters
----------
fileName : str
    Filename to store calibration parameters.

deviceManager : DeviceManager object
    realsense_device_manager object which manages connections to all cameras

chessboardHeight : int
    Number of chessboard intersections defining height of target chessboard

chessboardWidth : int
    Number of chessboard intersections defining width of target chessboard

chessboardSquareSize : float
    Dimension of side of chessboard (m)

Returns
-------
devicesTransformation : dict
    dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

Example
-----
    new('savedCalibration.cal')

## newIterative
```python
newIterative(fileName, deviceManager, cameraList, chessboardHeight, chessboardWidth, chessboardSquareSize)
```

New calibration parameters for each connected camera and are created and saved in a pickle file format.
Calibration parameters for each camera include a 4x4 transformation matrix and rmsd error of calibration

Parameters
----------
fileName : str
    Filename to store calibration parameters.

deviceManager : DeviceManager object
    realsense_device_manager object which manages connections to all cameras

cameraList : list
    list of serial numbers to calibrate cameras in order

chessboardHeight : int
    Number of chessboard intersections defining height of target chessboard

chessboardWidth : int
    Number of chessboard intersections defining width of target chessboard

chessboardSquareSize : float
    Dimension of side of chessboard (m)

Returns
-------
deviceTransformations : dict
    dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

Example
-----
    new('savedCalibration.cal')

## detectChessboard
```python
detectChessboard(deviceManager, cameraSet, chessboardHeight, chessboardWidth, chessboardSquareSize)
```

Chessboard locations are computed for each connected RealSense camera

Parameters
----------
deviceManager : DeviceManager object
    realsense_device_manager object which manages connections to all cameras

cameraSet : list
    list of camera serial numbers to detect the chessboard

chessboardHeight: int
    Number of chessboard intersections defining height of target chessboard

chessboardWidth: int
    Number of chessboard intersections defining width of target chessboard

chessboardSquareSize: float
    Dimension of side of chessboard (m)

Returns
-------
chessboardLocations : dict
    dictionary with keys of camera's serial number holding detected corners, local 2D and 3D coordinates of detected corners, and their valid depth points

Example
-----
    detectChessboard(deviceManager, chessboardHeight, chessboardWidth, chessboardSquareSize)

## poseTransformation
```python
poseTransformation(chessboardLocations, chessboardHeight, chessboardWidth, chessboardSquareSize)
```

Transformation matrices and error are computed for each connected RealSense camera

Parameters
----------
chessboardLocations : dict
    dictionary with keys of camera's serial number holding detected corners, local 2D and 3D coordinates of detected corners, and their valid depth points

chessboardHeight: int
    Number of chessboard intersections defining height of target chessboard

chessboardWidth: int
    Number of chessboard intersections defining width of target chessboard

chessboardSquareSize: float
    Dimension of side of chessboard (m)

Returns
-------
devicesTransformation: dict
    dictionary with keys of camera's serial number with each camera's 4x4 transformation matrix and root-mean squared error

Example
-----
    poseTransformation(chessboardLocations, chessboardHeight, chessboardWidth, chessboardSquareSize)

# dynamo.markers

## find_in_view
```python
find_in_view(deviceData)
```

Function which returns the location of all detected markers in camera view

Parameters
----------
deviceData : dict
    Dictionary with depth frame, infrared frame, depth sensor intrinsics, and transformation matrix

Returns
-------
markers : (n,3) array
    Array with rows representing each marker with columns denoting x,y,z locations

## detect
```python
detect(frame)
```

Function which detects all makers across all cameras in each frame

Parameters
----------
frame : dict
    Dictionary with keys as serial numbers of all connected cameras, containing each camera's saved images for the frame

Returns
-------
markers : (n,3) array
    Array with rows representing each marker with columns denoting x,y,z locations

# dynamo.stream

## captureThread
```python
captureThread(q, deviceManager, stream_time)
```

Function to capture frames from connected RealSense Cameras to memory

Parameters
----------
q : Queue object
    Queue for storing captured frames

deviceManager : DeviceManager object
    realsense_device_manager object which manages connections to all cameras

time : float
    Time in seconds for how long to collect data

## processThread
```python
processThread(q, devicesTransformation, saveDirectory)
```

Function to process frames from queue to save to disk in pickle format

Parameters
----------
q : Queue object
    Queue which stores captured frames

devicesTransformation : dict
    dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

folder : str
    Folder name in which to save frames, referenced to current working directory

iteration : int
    Iteration of data collection, subsequent iterations will be saved in the folder specified


## single
```python
single(deviceManager, devicesTransformation)
```

Function to return single, aligned pointcloud from conneted Realsense Cameras

Parameters
----------
deviceManager : DeviceManager object
    realsense_device_manager object which manages connections to all cameras

devicesTransformation : dict
    dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

Returns
-------
allPoints : (n,6) array
    array containing list of all points from all connected cameras


## start
```python
start(deviceManager, deviceTransformations, saveDirectory, stream_time)
```

Function to save aligned frames from connected RealSense Cameras to disk for future processing

Parameters
----------
deviceManager : DeviceManager object
    realsense_device_manager object which manages connections to all cameras

devicesTransformation : dict
    dictionary with keys of camera's serial number holding dictionary of calibration parameters per camera

folder : str
    Folder name in which to save frames, referenced to current working directory

iteration : int
    Iteration of data collection, subsequent iterations will be saved in the folder specified

time : float
    Time in seconds for how long to collect data

# dynamo.view

## depthFrametoPC
```python
depthFrametoPC(deviceData, **kwargs)
```

Function which takes saved depth and color/infrared 2D frames and converts to a colored 3D pointcloud

Parameters
----------
deviceData : dict
    Dictionary with depth frame, infrared frame, depth sensor intrinsics, and transformation matrix

Optional Arguments
------------------
format: str
    Format to define how to save pointcloud color information. Default is 'pcl', alternative is 'rgb'.

Returns
-------
pointCloud : (n,4) array
    Array with rows representing each point with columns denoting x,y,z, and PCL visualization color


## getPointCloud
```python
getPointCloud(frame)
```

Function which allows for the conversion of frame with multiple cameras into a pointcloud

Parameters
----------
frame : dict
    Dictionary with keys as serial numbers of each connected camera, with frames from each camera stored

Returns
-------
pointCloud : (n,4) array
    Array with rows representing each point with columns denoting x,y,z, and PCL visualization color


## viewPointClouds
```python
viewPointClouds(folderDirectory, full)
```

Function which allows for the viewing of pointClouds

Parameters
----------
folderDirectory : str
    Directory containing .pickle files of saved frames from capture

full : int
    0 if you wish to view every 10 frames, 1 if you wish to view every frame



# dynamo.realsense_device_manager

## enumerate_connected_devices
```python
enumerate_connected_devices(context)
```

Enumerate the connected Intel RealSense devices

Parameters:
-----------
context 	 	  : rs.context()
    The context created for using the realsense library

Return:
-----------
connect_device : array
    Array of enumerated devices which are connected to the PC


## post_process_depth_frame
```python
post_process_depth_frame(depth_frame, decimation_magnitude=1.0, spatial_magnitude=2.0, spatial_smooth_alpha=0.5, spatial_smooth_delta=20, temporal_smooth_alpha=0.4, temporal_smooth_delta=20)
```

Filter the depth frame acquired using the Intel RealSense device

Parameters:
-----------
depth_frame 	 	 	 : rs.frame()
    The depth frame to be post-processed
decimation_magnitude : double
    The magnitude of the decimation filter
spatial_magnitude 	 : double
                        The magnitude of the spatial filter
spatial_smooth_alpha	 : double
                        The alpha value for spatial filter based smoothening
spatial_smooth_delta	 : double
                        The delta value for spatial filter based smoothening
temporal_smooth_alpha : double
                        The alpha value for temporal filter based smoothening
temporal_smooth_delta : double
                        The delta value for temporal filter based smoothening


Return:
----------
filtered_frame : rs.frame()
The post-processed depth frame

## DeviceManager
```python
DeviceManager(self, context, pipeline_configuration)
```

