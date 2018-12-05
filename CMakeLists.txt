set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(acquisition)

#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation cnbiros_core cnbiros_tobi_msgs)
#find_package(PkgConfig)
#pkg_search_module(CNBILOOP REQUIRED cnbiloop)
#pkg_search_module(TOBICORE REQUIRED tobicore)
#pkg_search_module(TOBIID REQUIRED tobiid)
#pkg_search_module(TOBIIC REQUIRED tobiic)
#
#SET(CMAKE_BUILD_TYPE RelWithDebInfo)
#
#include_directories(${catkin_INCLUDE_DIRS} 
#					${CNBILOOP_INCLUDE_DIRS} 
#					${TOBICORE_INCLUDE_DIRS} 
#					${TOBIID_INCLUDE_DIRS} 
#					${TOBIIC_INCLUDE_DIRS} 
#					include)
#generate_messages(
#	DEPENDENCIES
#	std_msgs
#)
#
#catkin_package(
#  INCLUDE_DIRS 
#	include
#	${CNBILOOP_INCLUDE_DIRS}
#	${TOBICORE_INCLUDE_DIRS}
#	${TOBIID_INCLUDE_DIRS}
#	${TOBIIC_INCLUDE_DIRS}
#  LIBRARIES 
#    ${CNBILOOP_LIBRARIES}
#    ${TOBICORE_LIBRARIES}
#    ${TOBIID_LIBRARIES}
#    ${TOBIIC_LIBRARIES}
#	${PROJECT_NAME}
#  CATKIN_DEPENDS
#  	roscpp
#	message_runtime
#  DEPENDS
#	CNBILOOP
#	TOBICORE
#	TOBIID
#	TOBIIC
#)
#
#add_library(cnbiros_bci
#			src/TobiInterface.cpp
#			src/TidInterface.cpp
#			src/TidTools.cpp
#			src/TicInterface.cpp
#			src/TicTools.cpp
#			src/rostid.cpp
#			)
#add_dependencies(cnbiros_bci cnbiros_bci_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cnbiros_bci ${catkin_LIBRARIES} 
#								  ${CNBILOOP_LIBRARIES} 
#								  ${TOBICORE_LIBRARIES}
#								  ${TOBIID_LIBRARIES}
#								  ${TOBIIC_LIBRARIES})
#
#add_executable(rostid src/rostid.cpp)
#target_link_libraries(rostid cnbiros_bci)
#
#add_executable(rostic src/rostic.cpp)
#target_link_libraries(rostic cnbiros_bci)
