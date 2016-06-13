#/*
#Copyright 2016 Chris Papenfuß
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.
#*/
cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake-Modules" ${CMAKE_MODULE_PATH} )

SET(CMAKE_MFC_FLAG 2)

project( magicPresenter )

# Find Packages
find_package( PCL 1.8 REQUIRED )
#find_package( OpenCV REQUIRED )
find_package( OpenCV2 REQUIRED )
find_package( KinectSDK2 REQUIRED )

if( PCL_FOUND AND OPENCV_FOUND AND KinectSDK2_FOUND )
  add_executable( magicPresenter kinect_pcl_grabber.h detection-utils.h image-detection.h screen/ScreenImage.h screen/ScreenImage.cpp main.cpp )
  #add_executable( magicPresenter screen/stdafx.cpp screen/stdafx.h)
  
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${OPENCVSDK_INCLUDE_DIRS} )
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( -DDUO_GRABBER )
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${OPENCVSDK_LIBRARY_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( magicPresenter ${PCL_LIBRARIES} )
  target_link_libraries( magicPresenter ${OPENCVSDK_LIBRARIES} )
  target_link_libraries( magicPresenter ${KinectSDK2_LIBRARIES} )
endif()

