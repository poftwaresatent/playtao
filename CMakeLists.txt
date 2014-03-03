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

project (playtao)
#set (CMAKE_VERBOSE_MAKEFILE ON)
#set (CMAKE_BUILD_TYPE Debug)

##################################################
# OS detection

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
    include_directories (
      /opt/local/include
      /opt/local/include/eigen2
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /opt/local/include
      /opt/local/include/eigen2
      )
    link_directories (/opt/local/lib)
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
    include_directories (/usr/include/eigen2)
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen2)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    include_directories (
      /usr/local/include
      /usr/local/include/eigen2
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen2)
    link_directories (/usr/local/lib)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

##################################################

include (CheckCXXCompilerFlag)
check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

include (FindOpenGL)
if (NOT OPENGL_FOUND)
  message (FATAL_ERROR "OpenGL not found")
endif (NOT OPENGL_FOUND)

include (FindGLUT)
if (NOT GLUT_FOUND)
  message (FATAL_ERROR "GLUT not found")
endif (NOT GLUT_FOUND)

include (CheckIncludeFileCXX)

if (NOT EIGEN2_DIR)
  set (EIGEN2_DIR $ENV{EIGEN2_DIR})
endif (NOT EIGEN2_DIR)
if (EIGEN2_DIR)
  message ("using EIGEN2_DIR ${EIGEN2_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${EIGEN2_DIR}/include ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
  include_directories (
    ${EIGEN2_DIR}/include ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
else (EIGEN2_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    /usr/include/eigen2 /opt/local/include/eigen2)
  include_directories (/usr/include/eigen2 /opt/local/include/eigen2)
endif (EIGEN2_DIR)
check_include_file_cxx (Eigen/Core HAVE_EIGEN2)
if (${HAVE_EIGEN2})
  message ("found Eigen")
else (${HAVE_EIGEN2})
  message (FATAL_ERROR "Eigen not found. Install it in a system location (e.g. `sudo apt-get install libeigen2-dev') or pass -DEIGEN2_DIR:path=/path/to/eigen2 to cmake.")
endif (${HAVE_EIGEN2})

###########################################################
# check for jspace and tao
#
# Instead of two separate paths with all kinds of checks (as before ca
# 2011), just assume that if someone uses playtao, they also build
# stanford_wbc form source. That's the single use case still around in
# 2014 anyway.

if (NOT WBC_DIR)
  set (WBC_DIR $ENV{WBC_DIR})
endif (NOT WBC_DIR)
if (WBC_DIR)
  message ("using WBC_DIR ${WBC_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${WBC_DIR}/jspace ${WBC_DIR}/tao)
  include_directories (${WBC_DIR}/jspace ${WBC_DIR}/tao)
  link_directories (${WBC_DIR}/build/jspace ${WBC_DIR}/build/tao)
endif (WBC_DIR)

check_include_file_cxx (jspace/Model.hpp HAVE_JSPACE)
if (${HAVE_JSPACE})
  message ("found jspace")
else (${HAVE_JSPACE})
  message (FATAL_ERROR "jspace not found, try passing -DWBC_DIR:path=/path/to/stanford_wbc (assumes you have a build/ directory in there)")
endif (${HAVE_JSPACE})

check_include_file_cxx (tao/dynamics/taoNode.h HAVE_TAO)
if (${HAVE_TAO})
  message ("found tao")
else (${HAVE_TAO})
  message (FATAL_ERROR "tao not found, try passing -DWBC_DIR:path=/path/to/stanford_wbc (assumes you have a build/ directory in there)")
endif (${HAVE_TAO})

include_directories (
  .
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR})

file (GLOB SOURCES gfx/*.cpp gfx/*.c *.cpp)
list (APPEND LIBS ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} jspace_test)

add_executable (playtao ${SOURCES})
target_link_libraries (playtao ${LIBS})
