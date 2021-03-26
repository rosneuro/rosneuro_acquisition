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

set(LIB_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

set(LIB_SOURCES 
	src/Device.cpp
	src/DummyDevice.cpp
	src/Acquisition.cpp
	src/FactoryDevice.cpp
	src/DeviceRegistration.cpp
	)

set(LIB_TARGET ${catkin_LIBRARIES})

if(EEGDEV_LIBRARIES AND EEGDEV_INCLUDE_DIRS)
	set(LIB_INCLUDE_DIRS ${LIB_INCLUDE_DIRS} ${EEGDEV_INCLUDE_DIRS})
	set(LIB_SOURCES ${LIB_SOURCES} src/EGDDevice.cpp)
	set(LIB_TARGET ${LIB_TARGET} ${EEGDEV_LIBRARIES})
	message(STATUS "Found EEGDEV libraries in ${EEGDEV_LIBRARIES}")
	message(STATUS "Found EEGDEV headers in ${EEGDEV_INCLUDE_DIRS}")
else()
	message(WARNING "EEGDEV libraries not found. Functionalities will not be included")
endif()

if(LSL_LIBRARIES AND LSL_INCLUDE_DIRS)
	set(LIB_INCLUDE_DIRS ${LIB_INCLUDE_DIRS} ${LSL_INCLUDE_DIRS})
	set(LIB_SOURCES ${LIB_SOURCES} src/LSLDevice.cpp)
	set(LIB_TARGET ${LIB_TARGET} ${LSL_LIBRARIES})
	message(STATUS "Found LSL libraries in ${LSL_LIBRARIES}")
	message(STATUS "Found LSL headers in ${LSL_INCLUDE_DIRS}")
else()
	message(WARNING "LSL libraries not found. Functionalities will not be included")
endif()


include_directories(${LIB_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} ${LIB_SOURCES})

target_link_libraries(${PROJECT_NAME} ${LIB_TARGET})

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(acquisition src/acquisition.cpp)
target_link_libraries(acquisition rosneuro_acquisition)

add_executable(test_factory src/test_factory.cpp)
target_link_libraries(test_factory rosneuro_acquisition)

if(LSL_LIBRARIES AND LSL_INCLUDE_DIRS)
	add_executable(lslclient src/lslclient.cpp)
	target_link_libraries(lslclient rosneuro_acquisition)
endif()

if(EEGDEV_LIBRARIES AND EEGDEV_INCLUDE_DIRS)
	add_executable(test_egddevice src/test_egddevice.cpp)
	target_link_libraries(test_egddevice rosneuro_acquisition)
endif()
