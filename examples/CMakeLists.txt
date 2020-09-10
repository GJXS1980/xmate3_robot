find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
INCLUDE_DIRECTORIES(
    /usr/include/eigen3
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/inc/include
    /opt/ros/kinetic/include
    /usr/include/opencv2
    ${OpenCV_INCLUDE_DIRS}
)



LINK_DIRECTORIES(
    /usr/lib/x86_64-linux-gun
    ${PROJECT_SOURCE_DIR}/lib
    /opt/ros/kinetic/lib
)

# add_executable(example.cpp)

set(EXAMPLES
  #main
  joint_position_move
  cartesian_impedance_move
  cartesian_position_move
  cartesian_nullspace_move
  cartesian_rotation_move
  joint_impedance_move
  torque_control
  joint_s
  joint_s_mu
  cartesian_s_line_mu
  cartesian_s_line
  move_test
  arpose_grasp_demo
  io_test
  ros
  test
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  TARGET_LINK_LIBRARIES(${example}  
    RCI_Client.a
    xMateModel.a
    orocos-kdl.a
    glog.a
    pthread
    libroscpp.so
    librosconsole.so
    librostime.so
    libcpp_common.so
    libroscpp_serialization.so
    librosbag.so
    librosbag_storage.so
    ${OpenCV_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${image_transport_LIBRARIES}
) 
endforeach()
