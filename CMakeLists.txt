cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_acquisition)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_data
			 rosneuro_msgs)

find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
find_library(EEGDEV_LIBRARIES eegdev)
find_path(EEGDEV_INCLUDE_DIRS eegdev.h)

find_library(LSL_LIBRARIES lsl64)
find_path(LSL_INCLUDE_DIRS lsl_cpp.h)

message(STATUS "Found LSL libraries in ${LSL_LIBRARIES}")
message(STATUS "Found LSL headers in ${LSL_INCLUDE_DIRS}")

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_data
	rosneuro_msgs

  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
					${EEGDEV_INCLUDE_DIRS}
					${LSL_INCLUDE_DIRS}
					include
)

add_library(${PROJECT_NAME}
	src/Device.cpp
	src/EGDDevice.cpp
	src/DummyDevice.cpp
	#src/FactoryDevice.cpp
	#src/Acquisition.cpp
	src/LSLDevice.cpp
	src/SelfFactoryDevice.cpp
	src/DeviceRegistration.cpp
	#src/NeuroDataTools.cpp
	#src/Writer.cpp
	#src/XDFWriter.cpp
	#src/DummyWriter.cpp
	#src/FactoryWriter.cpp
			#src/Recorder.cpp
)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES} 
					  ${EEGDEV_LIBRARIES}
					  ${LSL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
#add_executable(acquisition src/acquisition.cpp)
#target_link_libraries(acquisition rosneuro_acquisition)

#add_executable(test_devicefactory src/test_devicefactory.cpp)
#target_link_libraries(test_devicefactory rosneuro_acquisition)

#add_executable(test_egddevice src/test_egddevice.cpp)
#target_link_libraries(test_egddevice rosneuro_acquisition)


add_executable(test_selffactory src/test_selffactory.cpp)
target_link_libraries(test_selffactory rosneuro_acquisition)

add_executable(test_regexp src/test_regexp.cpp)
target_link_libraries(test_regexp rosneuro_acquisition)

add_executable(lslclient src/lslclient.cpp)
target_link_libraries(lslclient rosneuro_acquisition)

