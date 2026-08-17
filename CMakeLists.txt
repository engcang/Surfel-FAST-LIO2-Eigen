cmake_minimum_required(VERSION 3.16)
project(surfel_fast_lio2_eigen LANGUAGES C CXX)

### Set compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

### Configure dependencies
#Codex: PCL exports optional OpenMP flags even though this target uses TBB only.
set(CMAKE_DISABLE_FIND_PACKAGE_OpenMP TRUE)

### Select ROS wrapper
if(DEFINED CATKIN_DEVEL_PREFIX)
    add_subdirectory(ros1)
elseif(DEFINED ENV{AMENT_PREFIX_PATH})
    add_subdirectory(ros2)
endif()
