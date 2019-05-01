# Multiple Camera Connection Guide

This guide includes instructions and suggestions for connecting multiple D400 series Intel RealSense cameras to a single computer.
Intel provides a [whitepaper](https://www.intel.com/content/www/us/en/support/articles/000028140/emerging-technologies/intel-realsense-technology.html) which details their experiments in connecting multiple devices to one computer with a USB hub. 

## Required Bandwidth
The primary conclusion of the whitepaper is that ~1200 MBps of bandwidth is recommended for each device over a USB port, including overhead, to stream depth and color frames at 848x480 resolution and 90 fps (highest resolution and framerate).

While USB 3.0 specifications list up to 5 Gbps of speed, the whitepaper cited an independent study which notes that to have robust, continuous, streaming, its best to stay under 1200 MBps per camera. This limits us to one camera per independent USB 3.0 controller. Therefore, it is recommended to budget 5 Gbps listed speed per camera, assuming that robust continuous streaming speed will scale with the listed speed.

Listed speeds for each USB 3.0+ specification are listed below:
* USB 3.0 : 5 Gbps
* USB 3.1: 10 Gbps
* USB 3.2: 20 Gbps

## USB Controllers vs. Ports

It is important to keep in mind that the number of ports do not correlate to the number of each USB controllers on the device. 
On most modern computers, there may only be one or two on-board USB controllers and an internal hub which connects multiple ports to one controller. 
To determine the number of USB controllers present on the computer in Windows, reference the device manager. 
Under "Universal Serial Bus Controllers", the number of "Intel (R) USB 3.0 eXtensible Host Controller" is the number of independent USB controllers you have on your system.
Please refer to your manufacturer's specifications to determine the exact USB 3.0 specification you have on your computer. 

## Maximum Camera Connections

The maximum number of cameras connected to a system can be determined by multiplying the number of independent controllers by the exact USB 3.0+ specification speed present on the system, and then dividing by 5. 
If your system can connect more cameras than it has ports, you may use a powered USB hub to attach more cameras to the USB controller. 
Please keep in mind that the whitepaper recommends 2W of power per connected camera. 

## Extending USB Controllers through PCI-E or Thunderbolt

To increase the number of cameras you can connect to the system, you may attach additional USB controllers, either through PCI-E or Thunderbolt connections.
The advantage of using a Thunderbolt connect is a single-cable connection between multiple cameras and the computer. 
A Thunderbolt 3 connection might allow up to 8 cameras to be connected to it, if it features 40 Gbps speeds.
Please be aware that some computers with Thunderbolt 3 only feature upto 20 Gbps speeds; please refer to your computer's specifications to determine the amount of bandwidth over Thunderbolt 3.
If your computer has Thunderbolt 3 support, we recommend this [hub](https://www.startech.com/Cards-Adapters/USB-3.0/Hubs/thunderbolt-3-usb-3-1-hub~TB33A1C). 
This hub features 20 Gbps of USB bandwidth and can connect to 4 cameras on its own. 
The hub also features a Thunderbolt 3 passthrough, allowing for an additional 20 Gbps of bandwidth in the connection.
The passthrough can be used to connect additional cameras through the computer's internal USB hub, or another hub which can support an additional 4 cameras. 

