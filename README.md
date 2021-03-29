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
  
 ## Supported devices
 
 The currently supported devices and the related plugins are reported below:
 
 | Device | Driver | Plugin |
 |:------:|:-------:|:------:|
 | GDF file | free | rosneuro::EGDDevice |
 | BDF file | free | rosneuro::EGDDevice |
 | Biosemi  | free | rosneuro::EGDDevice |
 | Neurosky | free | rosneuro::EGDDevice |
 | TobiIA   | free | rosneuro::EGDDevice |
 | NeurOne  | free | rosneuro::EGDDevice |
 | g.USBamp | proprietary | rosneuro::EGDDevice |
 | gTecNet  | proprietary | rosneuro::EGDDevice |
 | BBT      | proprietary | rosneuro::EGDDevice |
 | Wearable Sensing | proprietary | rosneuro::EGDDevice |
 | Cognionics Quick-20 | proprietary | rosneuro::EGDDevice |
 | AntNeuro eego | proprietary | rosneuro::EGDDevice |
 | LabStreaming Layer | free | rosneuro::LSLDevice |

 
## Developing custom plugins
The development of additional plugins for the package is based on the [pluginlib](http://wiki.ros.org/pluginlib) provided by ROS. Please, refer to the official documentation for more details.

A ROS Neuro acquisition plugin can be implemented by creating a new class derived by rosneuro::Device. The father class rosneuro::Device contains pure virtual methods that needs to be implemented in order to instanciated the derived class. 

### Creating a custom plugin
For this example, we will create a custom plugin for a hypothetic PlugDevice. Create the header class of the new plugin in `rosneuro_acquisition_plugin_plugdevice/include/rosneuro_acquisition_plugin_plugdevice/PlugDevice.hpp`. 

```cpp
#ifndef ROSNEURO_ACQUISITION_PLUGIN_PLUGDEVICE_HPP
#define ROSNEURO_ACQUISITION_PLUGIN_PLUGDEVICE_HPP

#include <memory>
#include <errno.h>
#include <string.h>
#include <pluginlib/class_list_macros.h>
#include <rosneuro_acquisition/Device.hpp>

namespace rosneuro {

class PlugDevice : public Device {

	public:

		PlugDevice(NeuroFrame* frame);
		virtual ~DummyDevice(void);
		bool Setup(float framerate);
		bool Open(const std::string& devname, int samplerate);
		bool Close(void);
		bool Start(void);
		bool Stop(void);
		size_t Get(void);
		size_t GetAvailable(void);
};

PLUGINLIB_EXPORT_CLASS(rosneuro::PlugDevice, rosneuro::Device)

}

#endif
```
The code is straightforward, we created a new class `PlugDevice` derived from `Device` and we implemented the required pure virtual methods.
Let's look to few lines more in detail:
```cpp
#include <pluginlib/class_list_macros.h>
#include <rosneuro_acquisition/Device.hpp>
```
Here, we include the [pluginlib](http://wiki.ros.org/pluginlib) macros that will be used to register the plugin. Furthermore, we include the father class `rosneuro_acquisition/Device.hpp`.
```cpp
namespace rosneuro {
...
}
```
Here we include the plugin must be in the namespace `rosneuro`.

```cpp
PLUGINLIB_EXPORT_CLASS(rosneuro::PlugDevice, rosneuro::Device)
```
Here we register the `PlugDevice` class as a plugin by providing the fully-qualified type of the class (`rosneuro::PlugDevice`) and the fully-qualified type of the base class (`rosneuro::Device`).

Now, let's create the implementatio of the plugin in: `rosneuro_acquisition_plugin_plugdevice/src/PlugDevice.cpp`:

```cpp
#ifndef ROSNEURO_ACQUISITION_PLUGIN_PLUGDEVICE_CPP
#define ROSNEURO_ACQUISITION_PLUGIN_PLUGDEVICE_CPP

#include "rosneuro_acquisition_plugin_plugdevice/PlugDevice.hpp"

namespace rosneuro {

PlugDevice::PlugDevice(NeuroFrame* frame) : Device(frame) {
	this->name_ = "PlugDevice";
}

PlugDevice::~PlugDevice(void) {}

bool PlugDevice::Setup(float framerate) {
	printf("[%s] - Setup done\n", this->name_.c_str());
	return true;
}

bool PlugDevice::Open(const std::string& devname, int samplerate) {
	printf("[%s] - Device open\n", this->name_.c_str());
	return true;
}

bool PlugDevice::Close(void) {
	printf("[%s] - Device closed\n", this->name_.c_str());
	return true;
}

bool PlugDevice::Start(void) {
	printf("[%s] - Device started\n", this->name_.c_str());
	return true;
}

bool PlugDevice::Stop(void) {
	printf("[%s] - Device stopped\n", this->name_.c_str());
	return true;
}

size_t PlugDevice::Get(void) {
	printf("[%s] - Get\n", this->name_.c_str());
	return 1;
}

size_t PlugDevice::GetAvailable(void) {
	printf("[%s] - Get available data\n", this->name_.c_str());
	return 0;
}

}

#endif
```
As you probably notice, this is a dummy implementation. The real implementation will depends on the device API to open, close, start, stop, configure, and get samples from the amplifier. Notice, that if the device requires additional configuration parameters, the best solution is to use `ros::param` in the `rosneuro::PlugDevice::Setup()` method.

### Building the plugin
To build the plugin, just add the usual lines to the `rosneuro_acqusition_plugin_plugdevice/CMakeLists.txt` file:

```
include_directories(include)
add_library(${PROJECT} src/PlugDevice.cpp)
```
Notice that in the `CMakeLists.txt` we need to add also the dependency to [pluginlib](http://wiki.ros.org/pluginlib) and to the rosneuro packages:
```
find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 pluginlib
			 rosneuro_data
			 rosneuro_msgs
       rosneuro_acquisition)
```

### Making the plugin available in the ROSNeuro Toolchain
In order to make the new plugin available we will need to create a new plugin XML file and to updated the `rosneuro_acqusition_plugin_plugdevice/package.xml`.
Create the plugin XML file in: `rosneuro_acqusition_plugin_plugdevice/plugin_plugdevice.xml`:
```xml
<library path="lib/librosneuro_acquisition_plugin_plugdevice">
  <class type="rosneuro::PlugDevice" base_class_type="rosneuro::Device">
    <description>This is the ROSNeuro Acquisition PlugDevice plugin.</description>
  </class>
</library>
```
In the `library` tag, we need to set the relative path to the plugin library. In this case: `lib/librosneuro_acquisition_plugin_plugdevice`.
Then, we have to define the type of the plugin `rosneuro::PlugDevice` and of the base class `rosneuro::Device`. We can also add a human-readable description of the plugin. Notice that the value of type of plugin (`rosneuro::PlugDevice`) will be used as argument of the acquistion to dynamically load the desired plugin.

To export the plugin, add to the `rosneuro_acqusition_plugin_plugdevice/package.xml` with the following lines:
```
  <export>
	  <rosneuro_acquisition plugin="${prefix}/plugin_dummydev.xml"/>
  </export>
 ```
where the tag `rosneuro_acquisition` identify the target library where the plugin should be added to, and `plugin="${prefix}/plugin_dummydev.xml` identify the path to the XML plugin file created before.

Notice that in `rosneuro_acqusition_plugin_plugdevice/package.xml` we need to add also the dependency to [pluginlib](http://wiki.ros.org/pluginlib) and to the rosneuro packages:
```xml
  <depend>rosneuro_acquisition</depend>
  <depend>rosneuro_data</depend>
  <depend>rosneuro_msgs</depend>
  <exec_depend>message_runtime</exec_depend>
	
  <build_depend>pluginlib</build_depend>
  <exec_depend>pluginlib</exec_depend>
```
To verify that the plugin is effectively recongized, build the workspace, source the setup file and run the following command:
```
rospack plugins --attrib=plugin rosneuro_acquisition
```

### Using the new plugin in the acquisition
Finally, now it is possible to run the acquisition with the new plugin, as follows:
```
rosrun rosneuro_acquisition acquisition _devtype:="rosneuro::PlugDevice" [Additional parameters]
```
