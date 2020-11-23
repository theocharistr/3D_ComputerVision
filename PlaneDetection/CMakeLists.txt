cmake_minimum_required(VERSION 3.9)

project( ProjectName)

###############
# packages #
###############

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
###############
# executables #
###############

add_executable( planedetection src/LSQfitting.cpp )
target_link_libraries( planedetection ${OpenCV_LIBS} )

 