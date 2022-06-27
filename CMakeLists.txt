cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#add_definitions(-std=c++17)


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")

project(playback)

# Edit this line for your environments...
SET(PCL_DIR "D:/github_repository/vcpkg/installed/x64-windows/share/pcl")

# Check the version...
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})