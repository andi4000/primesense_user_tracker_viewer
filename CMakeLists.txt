cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)

include(FindGLUT)
include_directories(${GLUT_INCLUDE_DIR})

include_directories(${OPENNI_INCLUDEDIR} /usr/include/nite)

rosbuild_add_executable(user_tracker src/user_tracker.cpp 
									src/SceneDrawer.h
									src/SceneDrawer.cpp
									src/kbhit.h
									src/kbhit.cpp
									src/signal_catch.h
									src/signal_catch.cpp)
target_link_libraries(user_tracker ${OPENNI_LIBRARIES} ${GLUT_LIBRARIES})
