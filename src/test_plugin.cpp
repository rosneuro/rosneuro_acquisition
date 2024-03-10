#include <pluginlib/class_loader.h>
#include <Device.hpp>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<rosneuro::Device> plug_loader("rosneuro_acquisition", "rosneuro::Device");

  try
  {
    boost::shared_ptr<rosneuro::Device> devone = plug_loader.createInstance("rosneuro::ExtDeviceOne");
    boost::shared_ptr<rosneuro::Device> devtwo = plug_loader.createInstance("rosneuro::ExtDeviceTwo");

	devone->Who();
	devtwo->Who();
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}
