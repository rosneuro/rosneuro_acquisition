cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_acquisition)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ROSNEURO_MSGS_MINVERSION 1.5.0)
find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 pluginlib
			 rosneuro_data)

find_package(rosneuro_msgs ${ROSNEURO_MSGS_MINVERSION} REQUIRED)
find_package(GTest REQUIRED)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	pluginlib
	rosneuro_data
	rosneuro_msgs

  DEPENDS
)


###########
## Build ##
###########

catkin_add_gtest(test_device test/test_device.cpp)
catkin_add_gtest(test_acquisition test/test_acquisition.cpp)
target_link_libraries(test_device ${GTEST_BOTH_LIBRARIES} pthread gmock)
target_link_libraries(test_acquisition ${GTEST_BOTH_LIBRARIES} pthread gmock)
target_link_libraries(test_device ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(test_acquisition ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(test_acquisition ${GTEST_BOTH_LIBRARIES} pthread gmock)
include_directories(${GTEST_INCLUDE_DIRS} gmock pthread)
include_directories(${catkin_INCLUDE_DIRS} ${rosneuro_msgs_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME}	
			src/Device.cpp
			src/Acquisition.cpp
			)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
				 ${rosneuro_msgs_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(acquisition src/acquisition.cpp)
target_link_libraries(acquisition rosneuro_acquisition)

add_executable(test_plugin src/test_plugin.cpp)
target_link_libraries(test_plugin rosneuro_acquisition)

#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )

install(TARGETS acquisition
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
