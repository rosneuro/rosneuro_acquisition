# ROSNeuro Acquisition package

The package provides a generic interface for acquiring data from different devices. The interace accepts plugins that can be independently developed and dynamically loaded. Currently, the package provides plugins for [eegdev](https://neuro.debian.net/pkgs/libeegdev0.html) and for [LabStreamingLayer (LSL)](https://github.com/sccn/labstreaminglayer) devices ([rosneuro/rosneuro_acquisition_plugins](https://github.com/rosneuro/rosneuro_acquisition_plugins)).

## Requirements
rosneuro_acquisition has been tested with the following configuration:
- **Ubuntu 18.04.05 LTS Bionic Beaver** and **ROS Melodic**
- **Ubuntu 20.04.02 LTS Focal Fossa** and **ROS Noetic**

rosneuro_acquisition depends on the following @rosneuro packages:
- [rosneuro/rosneuro_msgs](https://github.com/rosneuro/rosneuro_msgs) 
- [rosneuro/rosneuro_data](https://github.com/rosneuro/rosneuro_data) 

## Usage
The package acquires data from external devices and publishes it as a **NeuroFrame** message ([rosneuro_msgs](https://github.com/rosneuro/rosneuro_msgs)). Each published message represents a chunk of data as `samples x channels`. This defines a **framerate** according to which the data is published to other modules. For instance, if the device provides data at 512 Hz (sampling rate) and the acquisition is set to have a framerate of 16 Hz, each **NeuroFrame** will contains 32 samples (x channels) of data. For more information about **NeuroFrame**, please refere to the [rosneuro_msgs](https://github.com/rosneuro/rosneuro_msgs) documentation.

The following command can be used to launch the acquisition (**TODO: Check/change the arguments**):
```
rosrun rosneuro_acquisition acquisition _devarg:=[DEVICE] _devtype:=[DEVTYPE] _framerate:=[FRAMERATE] _samplerate:=[SAMPLERATE] _reopen:=[True/False] _autostart:=[True/False]
```

### Published Topics
- /neurodata ([rosneuro_msgs/NeuroFrame](https://github.com/rosneuro/rosneuro_msgs))

### Parameters
~<name>/`devarg` (`string`) 
  
  *TODO*
  
~<name>/`devtype` (`string`, default: "rosneuro::EGDDEvice") 
  
  Full name of the plugin to be loaded (e.g., rosneuro::EGDDevice, rosneuro::LSLDevice)

~<name>/`framerate` (`int`, default: 16) 
  
  Framerate (in Hz) of the published NeuroFrame message (e.g., 32)

~<name>/`samplerate` (`int`, default: depends on device) 
  
  Sampling rate (in Hz) of the acquisition device (if the device allows to set the sampling rate) (e.g., 512)

~<name>/`reopen` (`bool`, default: True) 
  
  Try to automatically re-open the device if closed

~<name>/`autostart` (`bool`, default: True) 
  
  Automatically start the acquisition after launch

### Services

~<name>/`start` (`std_srvs/Empty`)
  
  Start the acquisition device
  
 ~<name>/`stop` (`std_srvs/Empty`)
  
  Stop the acquisition device
  
 ~<name>/`quit` (`std_srvs/Empty`)
  
  Quit the acquisition
  
 ~<name>/`get_info` (`rosneuro_msgs/GetAcquisitionInfo`)
  
  Get the configuration of the device
