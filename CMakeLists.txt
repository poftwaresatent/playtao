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

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
    list (APPEND CMAKE_REQUIRED_INCLUDES /opt/local/include)
    include_directories (/opt/local/include)
    link_directories (/opt/local/lib)
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/local/include)
    include_directories (/usr/local/include)
    link_directories (/usr/local/lib)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

include (FindOpenGL)
if (NOT OPENGL_FOUND)
  message (FATAL_ERROR "OpenGL not found")
endif (NOT OPENGL_FOUND)

include (FindGLUT)
if (NOT GLUT_FOUND)
  message (FATAL_ERROR "GLUT not found")
endif (NOT GLUT_FOUND)

include (CheckIncludeFileCXX)

if (NOT JSPACE_DIR)
  set (JSPACE_DIR $ENV{JSPACE_DIR})
endif (NOT JSPACE_DIR)
if (JSPACE_DIR)
  message ("using JSPACE_DIR ${JSPACE_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${JSPACE_DIR}/include ${JSPACE_DIR} ${JSPACE_DIR}/jspace)
  include_directories (${JSPACE_DIR}/include ${JSPACE_DIR} ${JSPACE_DIR}/jspace)
  link_directories (${JSPACE_DIR}/lib ${JSPACE_DIR} ${JSPACE_DIR}/jspace
    ${JSPACE_DIR}/build ${JSPACE_DIR}/build/jspace)
endif (JSPACE_DIR)
#check_include_file_cxx (jspace/Model.hpp HAVE_JSPACE)
#if (${HAVE_JSPACE})
#  message ("found jspace")
#else (${HAVE_JSPACE})
#  message (FATAL_ERROR "jspace not found, try passing -DJSPACE_DIR:path=/path/to/jspace")
#endif (${HAVE_JSPACE})

if (NOT TAO_DIR)
  set (TAO_DIR $ENV{TAO_DIR})
endif (NOT TAO_DIR)
if (TAO_DIR)
  message ("using TAO_DIR ${TAO_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${TAO_DIR}/include ${TAO_DIR} ${TAO_DIR}/tao)
  include_directories (${TAO_DIR}/include ${TAO_DIR} ${TAO_DIR}/tao)
  link_directories (${TAO_DIR}/lib ${TAO_DIR} ${TAO_DIR}/tao)
endif (TAO_DIR)
#check_include_file_cxx (tao/dynamics/taoNode.h HAVE_TAO)
#if (${HAVE_TAO})
#  message ("found tao")
#else (${HAVE_TAO})
#  message (FATAL_ERROR "tao not found, try passing -DTAO_DIR:path=/path/to/tao")
#endif (${HAVE_TAO})

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

include_directories (
  .
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR})

file (GLOB SOURCES gfx/*.cpp gfx/*.c *.cpp)
list (APPEND LIBS ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} jspace_test)

add_executable (playtao ${SOURCES})
target_link_libraries (playtao ${LIBS})
