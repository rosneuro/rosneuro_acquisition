# ROSNeuro Acquisition package

The package provides a generic interface for acquiring data from different devices. The interace accepts plugins that can be independently developed. Currently, the package provides plugins for [eegdev](https://neuro.debian.net/pkgs/libeegdev0.html) and for [LabStreamingLayer (LSL)](https://github.com/sccn/labstreaminglayer) devices ([rosneuro/rosneuro_acquisition_plugins](https://github.com/rosneuro/rosneuro_acquisition_plugins)).

## Requirements
rosneuro_acquisition has been tested with the following configuration:
- **Ubuntu 18.04.05 LTS Bionic Beaver** and **ROS Melodic**
- **Ubuntu 20.04.02 LTS Focal Fossa** and **ROS Noetic**

## Usage
The package acquires data from external devices and publishes it as NeuroFrame message (rosneuro_messages/NeuroFrame.msg). Each published message represents a chunk of data as `samples x channels`. This defines a framerate according to which the data is published to other modules. For instance, if the device provides data at 512 Hz (sampling rate) and the acquisition is set to have a framerate of 16 Hz, each NeuroFrame will contains 32 samples (x channels) of data. For more information about [NeuroFrame](https://github.com/rosneuro/rosneuro_messages), please refere to the related documentation.
