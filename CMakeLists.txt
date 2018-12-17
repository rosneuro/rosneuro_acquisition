cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_acquisition)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_msgs)

find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
find_library(EEGDEV_LIBRARIES eegdev)
find_path(EEGDEV_INCLUDE_DIRS eegdev.h)


catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_msgs

  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
					${EEGDEV_INCLUDE_DIRS}
					include
)

add_library(${PROJECT_NAME}
			src/Device.cpp
			src/EGDDevice.cpp
			src/DummyDevice.cpp
			src/DeviceFactory.cpp
			src/Acquisition.cpp
			src/AcquisitionConverter.cpp
)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES} 
					  ${EEGDEV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(acquisition src/acquisition.cpp)
target_link_libraries(acquisition rosneuro_acquisition)

add_executable(test_devicefactory src/test_devicefactory.cpp)
target_link_libraries(test_devicefactory rosneuro_acquisition)

add_executable(test_egddevice src/test_egddevice.cpp)
target_link_libraries(test_egddevice rosneuro_acquisition)

#add_executable(test_rule1 src/test_rule1.cpp)
#target_link_libraries(test_rule1 rosneuro_acquisition)
#
#add_executable(test_rule2 src/test_rule2.cpp)
#target_link_libraries(test_rule2 rosneuro_acquisition)
#
#add_executable(test_rule3 src/test_rule3.cpp)
#target_link_libraries(test_rule3 rosneuro_acquisition)


