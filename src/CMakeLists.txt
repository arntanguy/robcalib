ADD_LIBRARY(robcalib SHARED logging.cpp calib_data.cpp calib_json.cpp maths.cpp)
target_link_libraries(robcalib ${Boost_LIBRARIES})
PKG_CONFIG_USE_DEPENDENCY(robcalib jsoncpp)
PKG_CONFIG_APPEND_LIBS(robcalib)
target_link_libraries (robcalib ${CMAKE_THREAD_LIBS_INIT})

# Build ROS module if ROS has been found on system
if(${ROSCPP_FOUND})
  ADD_LIBRARY(robcalib_ros SHARED input_module.cpp ros_input_module.cpp)
  PKG_CONFIG_USE_DEPENDENCY(robcalib_ros roscpp)
  PKG_CONFIG_USE_DEPENDENCY(robcalib_ros tf2_ros)
  PKG_CONFIG_APPEND_LIBS(robcalib_ros)

  SET(EXAMPLE_NAME live_calib_node)
  ADD_EXECUTABLE(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)
  TARGET_LINK_LIBRARIES(${EXAMPLE_NAME} robcalib_ros)
  PKG_CONFIG_USE_DEPENDENCY(${EXAMPLE_NAME} roboptim-core)
  target_link_libraries(${EXAMPLE_NAME} robcalib)
endif()

