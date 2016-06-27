ADD_EXECUTABLE(test_link_eye test_link_eye.cpp)
TARGET_LINK_LIBRARIES(test_link_eye
   ${Boost_FILESYSTEM_LIBRARY}
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} robcalib)
PKG_CONFIG_USE_DEPENDENCY(test_link_eye roboptim-core)
ADD_TEST(NAME test_link_eye
         COMMAND test_link_eye
         WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests")

ADD_EXECUTABLE(test_kinematics test_kinematics.cpp)
TARGET_LINK_LIBRARIES(test_kinematics
   ${Boost_FILESYSTEM_LIBRARY}
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} robcalib
   )
PKG_CONFIG_USE_DEPENDENCY(test_kinematics roboptim-core)
ADD_TEST(NAME test_kinematics
         COMMAND test_kinematics
         WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests")
