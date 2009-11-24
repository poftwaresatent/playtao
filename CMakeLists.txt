# Play TAO --- a tool for the Stanford Whole-Body Control Framework
#              http://stanford-wbc.sourceforge.net/
#
# Copyright (c) 2009 Stanford University. All rights reserved.
#
# Code in the gfx/ subdirectory comes from various sources and is
# released under various open source licenses:
#   Copyright (c) 1993, 1994, Silicon Graphics, Inc.
#   Copyright (c) 2002-2008 Jamie Zawinski
#   Copyright (c) 2004 Swiss Federal Institute of Technology, Lausanne.

cmake_minimum_required (VERSION 2.6)

if ($ENV{WBC_ROOT} STREQUAL "")
  message (FATAL_ERROR "The WBC_ROOT environment variable is not set. Did you bootstrap WBC and set your environment? See the stanford-wbc/README file and/or http://sourceforge.net/apps/trac/stanford-wbc/wiki/Installation for instructions.")
endif ($ENV{WBC_ROOT} STREQUAL "")
include ($ENV{WBC_ROOT}/share/wbc.cmake)
wbc_getvars()

project (playtao)
set (CMAKE_VERBOSE_MAKEFILE ON)
add_definitions (-DHAVE_LOG4CXX)

if (WIN32)
  message ("Detected Microsoft Windows")
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    message ("Detected Mac OS X")
    add_definitions (-DOSX)
    # add the default macports location to include and link path
    include_directories (/opt/local/include)
    link_directories (/opt/local/lib)
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    message ("Detected Linux")
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    message ("Detected OpenBSD")
    add_definitions (-DOPENBSD)
    include_directories (/usr/local/include)
    link_directories (/usr/local/lib)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

include (CheckCXXCompilerFlag)
check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

if (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)
if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
  if (CXX_FLAG_O0)
    add_definitions (-O0)
  endif (CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

include (FindOpenGL)
message ("DBG OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
message ("DBG OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")
if (NOT OPENGL_FOUND)
  message (FATAL_ERROR "GLUT not found")
endif (NOT OPENGL_FOUND)

include (FindGLUT)
message ("DBG GLUT_INCLUDE_DIRW: ${GLUT_INCLUDE_DIR}")
message ("DBG GLUT_LIBRARIES: ${GLUT_LIBRARIES}")
if (NOT GLUT_FOUND)
  message (FATAL_ERROR "GLUT not found")
endif (NOT GLUT_FOUND)

include_directories (${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${WBC_ROOT}/include ${PROJECT_SOURCE_DIR})
link_directories (${WBC_ROOT}/lib)

file (GLOB SOURCES gfx/*.cpp gfx/*.c *.cpp)
list (APPEND LIBS ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} Stanford_WBC)

if (NOT TRY_ROS)
  set (TRY_ROS true)
endif (NOT TRY_ROS)

if (TRY_ROS)

  message ("DBG ROS_ROOT: $ENV{ROS_ROOT}")
  if (NOT $ENV{ROS_ROOT} STREQUAL "")
    if (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      message ("WARNING ROS_ROOT is set but rosbuild.cmake not found")
      set (HAVE_ROS-NOTFOUND)
    else (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      set (HAVE_ROS true)
      include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      rosbuild_find_ros_package (urdf)
      message ("DBG urdf_PACKAGE_PATH: ${urdf_PACKAGE_PATH}")
      if (${urdf_PACKAGE_PATH} STREQUAL "")
	message ("WARNING urdf ROS package not found")
      else (${urdf_PACKAGE_PATH} STREQUAL "")
	# get compiler -I flags required for urdf and its deps
	rosbuild_invoke_rospack (urdf playtao temp cflags-only-I)
	include_directories (${playtao_temp})
	
	# get compiler -D flags required for urdf and its deps
	rosbuild_invoke_rospack (urdf playtao temp cflags-only-other)
	add_definitions (${playtao_temp})
	
	# get linker -L flags required for urdf and its deps
	rosbuild_invoke_rospack (urdf playtao temp libs-only-L)
	link_directories (${playtao_temp})
	
	# get linker -l flags required for urdf and its deps
	rosbuild_invoke_rospack (urdf playtao temp libs-only-l)
	list (APPEND LIBS ${playtao_temp})
	
	###	rosbuild_invoke_rospack (urdf playtao temp libs-only-other)
	
	add_definitions (-DHAVE_URDF)

      endif (${urdf_PACKAGE_PATH} STREQUAL "")
    endif (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  endif (NOT $ENV{ROS_ROOT} STREQUAL "")

  if (NOT HAVE_ROS)
    message ("ROS not found, cannot handle KDL")
  endif (NOT HAVE_ROS)

endif (TRY_ROS)

add_executable (playtao ${SOURCES})
target_link_libraries (playtao ${LIBS})
