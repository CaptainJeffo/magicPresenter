#.rst:
# FindOPENCV
# --------------

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(OPENCV_FOUND FALSE)
    unset(_DIR)
  endif()
endmacro()

# Check Files Macro
macro(CHECK_FILES _FILES _DIR)
  set(_MISSING_FILES)
  foreach(_FILE ${${_FILES}})
    if(NOT EXISTS "${_FILE}")
      get_filename_component(_FILE ${_FILE} NAME)
      set(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
    endif()
  endforeach()
  if(_MISSING_FILES)
    message(WARNING "In directory \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
    set(OPENCV_FOUND FALSE)
    unset(_FILES)
  endif()
endmacro()

# Target Platform
set(TARGET_PLATFORM)
if(NOT CMAKE_CL_64)
  set(TARGET_PLATFORM x86)
else()
  set(TARGET_PLATFORM x64)
endif()

##### Find OPENCV Motion #####

# Found
set(OPENCV_FOUND TRUE)
if(MSVC_VERSION LESS 1700)
  message(WARNING "OPENCV Motion SDK supported Visual Studio 2012 or later.")
  set(OPENCV_FOUND FALSE)
endif()

# Root Directoty
set(OPENCVSDK_DIR)
if(OPENCV_FOUND)
  set(OPENCVSDK_DIR $ENV{OPENCV_DIR} CACHE PATH "OPENCV Motion SDK Install Path." FORCE)
  check_dir(OPENCVSDK_DIR)
endif()

# Include Directories
set(OPENCVSDK_INCLUDE_DIRS)
if(OPENCV_FOUND)
  set(OPENCVSDK_INCLUDE_DIRS ${OPENCVSDK_DIR}/../../include)
  check_dir(OPENCVSDK_INCLUDE_DIRS)
endif()

# Library Directories
set(OPENCVSDK_LIBRARY_DIRS)
if(OPENCV_FOUND)
  set(OPENCVSDK_LIBRARY_DIRS ${OPENCVSDK_DIR}/lib)
  check_dir(OPENCVSDK_LIBRARY_DIRS)
endif()

# Dependencies
set(OPENCVSDK_LIBRARIES)
if(OPENCV_FOUND)
  set(OPENCVSDK_LIBRARIES ${OPENCVSDK_LIBRARY_DIRS}/opencv_world310.lib)
#  set(OPENCVSDK_LIBRARIES ${OPENCVSDK_LIBRARIES};${OPENCVSDK_LIBRARY_DIRS}/DUOLib.lib)

  check_files(OPENCVSDK_LIBRARIES OPENCVSDK_LIBRARY_DIRS)
endif()

message(STATUS "OPENCV_FOUND : ${OPENCV_FOUND}")