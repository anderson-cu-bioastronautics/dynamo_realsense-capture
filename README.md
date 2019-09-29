# Dynamic Body Shape and Motion Capture with Intel RealSense Cameras (DynaMo)

*Check out our paper on the Journal of Open Source Software!* [![DOI](https://joss.theoj.org/papers/10.21105/joss.01466/status.svg)](https://doi.org/10.21105/joss.01466)

DynaMo is an open-source Python package which allows for the capture of dynamic body shape data and/or motion capture using Intel RealSense cameras. 
The package is developed primarly for use in biomechanical analyses, but can be extended to any application where dynamic changes in surface morphology need to captured, or the 3D location of a marker needs to be tracked. 

## Features

### Dynamic Body Shape Capture

One or multiple cameras can be used to capture dyanamic changes in body shape through any motion. 
Multiple cameras, which are calibrated to a global coordinate system with this package, are recommended to get a full 3D view of the dynamic changes in body shape. 

*Example: Dynamic foot shape capture of gait on treadmill using DynaMo*
![Dynamic foot shape capture of gait on treadmill](documentation/dynafoot.gif)



### Supported Marker Tracking

During data collection with the cameras, markers can be used to obtain precise 3D locations of points of interest when collecting depth and infrared frames.
Reflective tape can be used as the markers due to its easy thresholding in the infrared frame. 
[Duck brand auto reflective tape](https://www.duckbrand.com/products/paint-diy-tapes/auto-reflective-tape/white-15-in-x-30-in) was successfully tested with this package. 
Markers must be a minimum of 1cm x 1cm to be reliabily detected by the included algorithms. 

## Supported Hardware
The package is designed for Intel RealSense D4XX cameras. 
This software has only been tested on Windows. 
We recommend at minimum for one camera:

* 4GB RAM
* 2.00 GHz Processor
* One USB 3.0 Port

Collecting data from the cameras is very processor and USB bandwidth heavy. 
If data is to be collected for a longer amount of time, a larger amount of RAM is recommended to store the data. 

We have used this code to successfully capture depth and infrared frames from 6 cameras at 90 fps, with a 848x480 resolution, using the following specifications:

* 16GB RAM
* Quad-Core 3.60 GHz Processor
* Cameras connected through Thunderbolt 3 USB Hub
  
To connect multiple cameras to the system, see this included [guide](documentation/multicamConnectionGuide.md).

## Dependencies

The package is designed to work with and was tested with Python 3.6.5.
Other Python 3.X versions may be supported, but will depend on dependency requirements and compatibilities. 

The package requires the following dependices (not included):

* numpy
* OpenCV 2
* pyrealsense2
* scikit-learn

An optional dependency is the python-pcl library, for viewing of the collected pointclouds with the included functions.
This library is not required to capture and save the data, and can be substituted in [view.py](view.py) for a pointcloud viewer of your choice. 
The PCL pointcloud viewer was selected for its quick response and ability to update the pointcloud in a single viewing window. 
This library is included in the Anaconda environment linked below. 
Since the Python wrapper for PCL's installation is highly dependent on your system configuration, please install the python-pcl wrappers using instructions found online for your specific system. 

## Recommended Installation

We recommend using the Anaconda Package Manager to setup a Python 3.6.5 environment to use this package. 
If you wish to setup a new environment, an Anaconda [environment](environment.yml) file is included to automatically install most dependencies.
To setup a new environment for DynaMo, type the following command into either a terminal or the Anaconda prompt:
```
conda env create  -f=/path/to/environment.yml -n dynamo-env
```
This will create a new environment named `dynamo-env`, which can be activated by typing `conda activate dynamo-env`.  
Once the environment is setup and activated, pyrealsense2 can be installed by using `pip install pyrealsense2`. 

An optional dependency is [`pptk`](https://github.com/heremaps/pptk), which is used to view sample data, tutorial, and to test installation. 
`pptk` can be installed by using `pip install pptk`. 

Once you have your Python environment and dependencies setup, clone or download this repository and then run ``python setup.py install`` to install the package into your environment.

## How to use DynaMo

### Included Functions

The package contains the following functions

[realsense_device_manager.py](dynamo/realsense_device_manager.py) - Contains `Device Manager` Class which is used to manage and communicate with the connected RealSense devices. 

[calibration.py](dynamo/calibration.py) - Calibrates connected RealSense devices with a template chessboard to a single global coordinate system

[stream.py](dynamo/stream.py) - Allows for the saving of frames from connected RealSense devices to disk  

[view.py](dynamo/view.py) - Provides pointcloud viewing of the saved frames from disk

[marker_tracker.py](dynamo/marker_tracker.py) - Saves global marker locations to disk. 

### API Documentation

Please refer to the  [API Documentation](https://anderson-cu-bioastronautics.github.io/dynamo_realsense-capture/) for details on how to use the included functions. 


### Examples

The [examples](examples/examples.md) pages includes tutorials on how to use this package.

See this [notebook](tutorial.ipynb) for an explanation of the processes used in the included functions. 

### Testing

The package includes a script which tests the installation of the `DynaMo` package. 
After downloading the `DynaMo` package, run `python testscript.py` inside the [`sample_scripts`](sample_scripts) folder to view a sample scene using the `DynaMo` package.

Please note, you will require the `pptk` package to view the scene.

## Contributing

To report an issue or problem with DynaMo, please create a new [issue](https://github.com/anderson-cu-bioastronautics/dynamo_realsense-capture/issues). 
You may also contact [abhishektha](https://github.com/abhishektha) with any support or general questions about DynaMo. 
We also welcome any meaningful contributions to DynaMo. 
If you have code you would like to contribute, please create a [pull request](https://github.com/anderson-cu-bioastronautics/dynamo_realsense-capture/pulls) to submit your code. 

## License

This package is licensed under the Apache License, Version 2.0. Copyright 2019 Anderson Bioastronautics Research Group.

This package contains code derived from Intel's librealsense Python box_dimensioner_multicam example (https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples/box_dimensioner_multicam).
Modifications to this code are denoted in the source files.
See LICENSE-librealsense for the license pertaining to librealsense code. 

This package also redistributes ``calculate_rmsd.py`` from the [rmsd](https://github.com/charnley/rmsd) library. 
See LICENSE-rmsd for the license pertaining to ``calculate_rmsd.py``.




