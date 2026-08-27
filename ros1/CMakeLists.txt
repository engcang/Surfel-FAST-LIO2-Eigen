### Find dependencies
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    livox_ros_driver
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    tf2_ros
)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

catkin_package()

### Build node
add_executable(${PROJECT_NAME}_mapping src/main_node.cpp)
add_dependencies(${PROJECT_NAME}_mapping ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_mapping PRIVATE
    include
    src
    ../surfel_fast_lio2_eigen/include
    ../third_party
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mapping
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARY}
    ${PCL_FILTERS_LIBRARY}
    TBB::tbb
)

### Install
install(TARGETS ${PROJECT_NAME}_mapping
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros1
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES rviz.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros1
)
