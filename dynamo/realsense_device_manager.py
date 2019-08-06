##############################################################################################################################################
##                             License: Apache 2.0. See LICENSE and LICENSE.librealsense files in root directory.		                    ##
##############################################################################################################################################
## This code has been appended to and modified from the librealsense box_dimensioner_multicam example:                                      ##
## https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/box_dimensioner_multicam/realsense_device_manager.py ##
## Modified functions are denoted with *Modified* and new functions are denoted with *New*                                                  ##
##############################################################################################################################################
__doc__ = \
"""
Class for managing Intel RealSense Devices
This code has been appended to and modified from the librealsense box_dimensioner_multicam example:                                      
https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/box_dimensioner_multicam/realsense_device_manager.py 

Modified functions are denoted with *Modified* and new functions are denoted with *New*               

Distributed as a module of DynaMo: https://github.com/anderson-cu-bioastronautics/dynamo_realsense-capture
"""
import pyrealsense2 as rs


class Device:
    def __init__(self, pipeline, pipeline_profile):
        """
        Class to manage each Intel RealSense D4XX Device

        Parameters
        ----------
        pipeline : rs.pipeline() object

        pipeline_profile : enabled rs.pipeline() object

        """
        self.pipeline = pipeline
        self.pipeline_profile = pipeline_profile


def enumerate_connected_devices(context):
    """
    Enumerate the connected Intel RealSense devices

    Parameters
    -----------
    context : rs.context()
        The context created for using the realsense library

    Return
    -----------
    connect_device : array
        Array of enumerated devices which are connected to the PC

    """
    connect_device = []
    for d in context.devices:
        if d.get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device.append(d.get_info(rs.camera_info.serial_number))
    return connect_device


def post_process_depth_frame(depth_frame, decimation_magnitude=1.0, spatial_magnitude=2.0, spatial_smooth_alpha=0.5, spatial_smooth_delta=20, temporal_smooth_alpha=0.4, temporal_smooth_delta=20):
    """
    Filter the depth frame acquired using the Intel RealSense device

    Parameters
    -----------
    depth_frame : rs.frame()
        The depth frame to be post-processed

    decimation_magnitude : double
        The magnitude of the decimation filter

    spatial_magnitude : double
        The magnitude of the spatial filter

    spatial_smooth_alpha : double
        The alpha value for spatial filter based smoothening

    spatial_smooth_delta : double
        The delta value for spatial filter based smoothening

    temporal_smooth_alpha : double
        The alpha value for temporal filter based smoothening

    temporal_smooth_delta : double
        The delta value for temporal filter based smoothening


    Return:
    ----------
    filtered_frame : rs.frame()
    The post-processed depth frame
    """

    # Post processing possible only on the depth_frame
    assert (depth_frame.is_depth_frame())

    # Available filters and control options for the filters
    decimation_filter = rs.decimation_filter()
    spatial_filter = rs.spatial_filter()
    temporal_filter = rs.temporal_filter()

    filter_magnitude = rs.option.filter_magnitude
    filter_smooth_alpha = rs.option.filter_smooth_alpha
    filter_smooth_delta = rs.option.filter_smooth_delta

    # Apply the control parameters for the filter
    decimation_filter.set_option(filter_magnitude, decimation_magnitude)
    spatial_filter.set_option(filter_magnitude, spatial_magnitude)
    spatial_filter.set_option(filter_smooth_alpha, spatial_smooth_alpha)
    spatial_filter.set_option(filter_smooth_delta, spatial_smooth_delta)
    temporal_filter.set_option(filter_smooth_alpha, temporal_smooth_alpha)
    temporal_filter.set_option(filter_smooth_delta, temporal_smooth_delta)

    # Apply the filters
    filtered_frame = decimation_filter.process(depth_frame)
    filtered_frame = spatial_filter.process(filtered_frame)
    filtered_frame = temporal_filter.process(filtered_frame)

    return filtered_frame



class DeviceManager:
    def __init__(self, context, pipeline_configuration):
        """
        Class to manage all connected Intel RealSense devices

        Parameters
        -----------
        context : rs.context()
            The context created for using the realsense library

        pipeline_configuration 	: rs.config()
            The realsense library configuration to be used for the application

        """
        assert isinstance(context, type(rs.context()))
        assert isinstance(pipeline_configuration, type(rs.config()))
        self._context = context
        self._available_devices = enumerate_connected_devices(context)
        self._enabled_devices = {}
        self._config = pipeline_configuration
        self._frame_counter = 0


    def enable_device(self, device_serial, enable_ir_emitter):
        """
        Enable an Intel RealSense Device

        Parameters
        -----------
        device_serial : string
            Serial number of the realsense device

        enable_ir_emitter : bool
            Enable/Disable the IR-Emitter of the device

        """
        pipeline = rs.pipeline()

        # Enable the device
        self._config.enable_device(device_serial)
        pipeline_profile = pipeline.start(self._config)

        # Set the acquisition parameters

        sensor = pipeline_profile.get_device().first_depth_sensor()
        #sensor.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
        self._enabled_devices[device_serial] = (Device(pipeline, pipeline_profile))

    def disable_all_devices(self):
        """
        *New*
        Disable all connected Intel Realsense Devices

        """
        for (serial, device) in self._enabled_devices.items():
            device.pipeline.stop()
        self._config.disable_all_streams()

    def enable_all_devices(self, enable_ir_emitter=False):
        """
        Enable all the Intel RealSense Devices which are connected to the PC

        Parameters
        -----------
        enable_ir_emitter : bool
            Enable/Disable the IR-Emitter of the device

        """
        print(str(len(self._available_devices)) + " devices have been found")

        for serial in self._available_devices:
            self.enable_device(serial, enable_ir_emitter)
    
    def enable_device_from_file(self, file):
        """
        *New*
        Enable a device from a .bag file

        Parameters
        -----------
        file : str
            Filepath of .bag file from which to load a device
        """
        pipeline = rs.pipeline()

        # Enable the device
        self._config.enable_device_from_file(file)
        pipeline_profile = pipeline.start(self._config)

        self._enabled_devices[file] = (Device(pipeline, pipeline_profile))



    def enable_all_emitters(self):
        """
        *Modified*
        Enable/Disable the emitters of all the connected intel realsense device
        Modified to be done before activiating device, performed using rs.context
        """
        for device in self._context.devices:
            # Get the active profile and enable the emitter for all the connected devices
            sensor = device.first_depth_sensor()
            sensor.set_option(rs.option.emitter_enabled, 1)
            sensor.set_option(rs.option.laser_power, 360)

    def load_settings_json(self, path_to_settings_file):
        """
        *Modified*
        Load the settings stored in the JSON file

        This function is modified to obtain device from rs.context() and not from enabled devices

        Run before enabling devices to avoid bugs

        Parameters
        ----------
        path_to_settings_file : str
            Path to JSON settings file for all cameras

        """

        file = open(path_to_settings_file, 'r')
        json_text = file.read().strip()
        file.close()

        for device in self._context.devices:
            # Get the active profile and load the json file which contains settings readable by the realsense
            #device = device.pipeline_profile.get_device()
            advanced_mode = rs.rs400_advanced_mode(device)
            advanced_mode.load_json(json_text)

    def poll_frames(self):
        """
        *Modified*
        Poll for frames from the enabled Intel RealSense devices.
        This function is modified to return frame objects which are of their inherent format from the pyrealsense2 libray.

        Returns
        --------

        frameCollection : dict
            Dictionary with keys as serial numbers of all connected cameras, containing each camera's saved images for the frame
        """
        frameCollection = {}
        while len(frameCollection) != len(self._enabled_devices) :
            for (serial, device) in self._enabled_devices.items():
                if not serial in frameCollection:
                    #frameCollection[serial] = {}
                    pipeline = device.pipeline
                    frames = pipeline.poll_for_frames()
                    frames.keep()
                    if frames.size() != 0:
                        frameCollection[serial] = frames
        return frameCollection

    def get_depth_shape(self):
        """ 
        Returns width and height of the depth stream for one arbitrary device

        Returns
        -------

        width: int
        height: int
        """
        width = -1
        height = -1
        for (serial, device) in self._enabled_devices.items():
            for stream in device.pipeline_profile.get_streams():
                if (rs.stream.depth == stream.stream_type()):
                    width = stream.as_video_stream_profile().width()
                    height = stream.as_video_stream_profile().height()
        return width, height

    def get_device_intrinsics(self, frames):
        """
        Get the intrinsics of the imager using its frame delivered by the realsense device

        Parameters
        -----------
        frames : rs::frame
            The frame grabbed from the imager inside the Intel RealSense for which the intrinsic is needed

        Returns
        ------
        device_intrinsics : dict
            Dictionary with device_intrinsics stored as 

        keys  : serial
            Serial number of the device

        values: [key]
            Intrinsics of the corresponding device
        """
        device_intrinsics = {}
        for (serial, frameset) in frames.items():
            device_intrinsics[serial] = {}
            try:
                device_intrinsics[serial][rs.stream.depth] = frameset.get_depth_frame().get_profile().as_video_stream_profile().get_intrinsics()
            except:
                pass
            try:
                device_intrinsics[serial][rs.stream.color] = frameset.get_color_frame().get_profile().as_video_stream_profile().get_intrinsics()
            except:
                pass
            try:
                device_intrinsics[serial][rs.stream.infrared] = frameset.get_infrared_frame(1).get_profile().as_video_stream_profile().get_intrinsics()
            except:
                pass

        return device_intrinsics

    def get_depth_to_color_extrinsics(self, frames):
        """
        Get the extrinsics between the depth imager 1 and the color imager using its frame delivered by the realsense device

        Parameters
        -----------
        frames : rs::frame
            The frame grabbed from the imager inside the Intel RealSense for which the intrinsic is needed

        Return
        ------
        device_intrinsics : dict

        keys  : serial
            Serial number of the device

        values: [key]
            Extrinsics of the corresponding device
        """
        device_extrinsics = {}
        for (serial, frameset) in frames.items():
            device_extrinsics[serial] = frameset[rs.stream.depth].get_profile().as_video_stream_profile().get_extrinsics_to(
                frameset[rs.stream.color].get_profile())
        return device_extrinsics

    def disable_streams(self):
        """
        *New*
        Disable all streams in device manager's rs::config

        """
        self._config.disable_all_streams()


if __name__ == "__main__":
    try:
        c = rs.config()
        c.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
        c.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 6)
        c.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 6)
        c.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)
        device_manager = DeviceManager(rs.context(), c)
        device_manager.enable_all_devices()
        for k in range(150):
            frames = device_manager.poll_frames()
        device_manager.enable_emitter(True)
        device_extrinsics = device_manager.get_depth_to_color_extrinsics(frames)
    finally:
        device_manager.disable_streams()
