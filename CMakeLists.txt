# Copyright 2016 Arnaud TANGUY 
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.8 to configure the package.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME robcalib)
SET(PROJECT_DESCRIPTION "Full robot calibration")
SET(PROJECT_URL "")
OPTION(BUILD_TESTS "Build unit tests" ON)

SET(CXX_DISABLE_WERROR TRUE)
ADD_DEFINITIONS(-std=c++11)

SETUP_PROJECT()

SEARCH_FOR_EIGEN()
ADD_REQUIRED_DEPENDENCY("roboptim-core")

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
SET(BOOST_COMPONENTS system filesystem log program_options)
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)

find_package(Threads)

add_required_dependency(jsoncpp)
add_optional_dependency(roscpp)
if(${ROSCPP_FOUND})
  set(DEFINE_HAS_ROS_MACRO "#define ROBCALIB_HAS_ROS 1")
  add_required_dependency(tf2_ros)
else()
  set(DEFINE_HAS_ROS_MACRO "")
endif()

INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(include/Sophus)
INCLUDE_DIRECTORIES(${JSONCPP_INCLUDE_DIRS})
ADD_SUBDIRECTORY(src)

if(BUILD_TESTS)
  ENABLE_TESTING()
  ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK) 
  find_package(Boost COMPONENTS ${BOOST_COMPONENTS} unit_test_framework REQUIRED)
  ADD_SUBDIRECTORY(tests)
endif()

SETUP_PROJECT_FINALIZE()
