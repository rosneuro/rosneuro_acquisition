set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_acquisition)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
find_library(EEGDEV_LIBRARIES eegdev)
find_path(EEGDEV_INCLUDE_DIRS eegdev.h)

include_directories(${catkin_INCLUDE_DIRS} 
					${EEGDEV_INCLUDE_DIRS}
					include)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	message_runtime
  DEPENDS
)

add_library(rosneuro_acquisition
			src/Device.cpp
			src/EGDDevice.cpp
			src/DummyDevice.cpp
			src/DeviceFactory.cpp
			src/Acquisition.cpp
			)
add_dependencies(rosneuro_acquisition rosneuro_acquisition_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosneuro_acquisition ${catkin_LIBRARIES} ${EEGDEV_LIBRARIES})

add_executable(acquisition src/acquisition.cpp)
target_link_libraries(acquisition rosneuro_acquisition)

add_executable(test_devicefactory src/test_devicefactory.cpp)
target_link_libraries(test_devicefactory rosneuro_acquisition)

add_executable(test_egddevice src/test_egddevice.cpp)
target_link_libraries(test_egddevice rosneuro_acquisition)

add_executable(test_rule1 src/test_rule1.cpp)
target_link_libraries(test_rule1 rosneuro_acquisition)

add_executable(test_rule2 src/test_rule2.cpp)
target_link_libraries(test_rule2 rosneuro_acquisition)

add_executable(test_rule3 src/test_rule3.cpp)
target_link_libraries(test_rule3 rosneuro_acquisition)


