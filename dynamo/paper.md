---
title: 'DynaMo: Dynamic Body Shape and Motion Capture with Intel RealSense Cameras'
tags:
    - Python
    - biomechanics
    - body shape
    - motion capture
authors:
    - name: Abhishektha Boppana
      orcid:
      affiliation: 1
    - name: Allison P. Anderson
      orcid:
      affiliation: 1
affiliations:
    - name: Ann and H.J. Smead Department of Aerospace Engineering Sciences, University of Colorado Boulder
    - index: 1
date: 

---

# Background 

Human body shape has been previous captures with a variety of methodologies, including laser lines, structured light, photogrammetry, and millimeter waves (cite). 
However these technologies require expensive modules and have limited ability to capture dynamic changes in body shape. 

Similarly, motion capture with specific markers is commonly done through camera-based motion tracking (cite vicon)
Specific systems for marker tracking are also very cost prohibitive. 

Recently, Intel has released the D415 and D435 RealSense Depth Cameras, which use near-infrared structured light patterns and two infrared imagers to capture depth information at up to 90 frames per second.
Purchasing a set of these cameras is much more affordable than buying a dedicated motion capture system for shape or marker tracking.

While Intel provides the ``librealsense`` (cite) library to interface with their cameras, tools are not provided to use multiple devices to collect shape and marker-tracking information.

# Summary

``DynaMo`` is a Python library which provides tools to capture dynamic changes in body shape and track locations of markers using Intel RealSense D4xx cameras. 
``DynaMo`` consists of a number of scripts which allow for calibration of multiple RealSense D4XX cameras to a common global coordinate system, simultaneous streaming of multiple RealSense D4XX cameras, and viewing of data from multiple RealSense D4XX cameras in pointcloud format. 
The library is optimized to reduce the number of dropped frames while streaming 

``DynaMo`` was developed from the examples provided by Intel in the Python ``librealsense`` (cite) library. It has been successfully tested on streaming up to six cameras connected to one computer. 
Scripts provided with the ``DynaMo`` library allow for the easy setup, calibration, streaming, and viewing of camera data. 
Connected cameras are setup using a ``device_manager`` object which handles calls for communicating with the cameras. 
Cameras are first calibrated to a common global coordinate system by using a defined chessboard.
The chessboard must be viewable by all cameras. 
Streaming is achieved by reading frames from each camera into a dictionary object saved in the computer's RAM. 
Once streaming is complete, ``DynaMo`` aligns the images collected by the sensors in each camera and saves the images as ``pickle`` objects to the disk. 
The data from all cameras can then be viewed as a single pointcloud for each frame with the included script. 
In addition, reflective markers can be isolated and tracked from the pointclouds.

``DynaMo`` is designed to primarily assist those in the biomechanics and medical field in capturing motion capture or body shape data. 
It is currently being used in the Anderson Bioastronautics Research Group to capture dynamic changes in foot morphology. 

# Acknowledgments
This work was supported by a National Science Foundation Graduate Research under grant DGE 1650115. The authors would like to thank Dr. Rodger Kram and Dr. Wouter Hoogkamer for the use of their laboratory space for development and testing of the package. 
