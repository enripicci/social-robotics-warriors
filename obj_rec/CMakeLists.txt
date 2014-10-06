cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

set(PROJECTNAME obj_rec)
PROJECT(${PROJECTNAME})

set(ROSBUILD_DONT_REDEFINE_PROJECT true)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_executable(setup src/setup.cpp)
rosbuild_add_executable(recognition src/recognition.cpp src/kdtree.c src/Blob.cpp src/ColorTable.cpp src/DataFitting.cpp src/Object.cpp)
target_link_libraries(recognition gsl blas)
rosbuild_add_executable(percept3d src/percept3d.cpp)
