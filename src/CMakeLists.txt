add_executable(rsgm_ros 
    main.cpp
    rsgm_ros.cpp
)

include_directories(rsgm_ros
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
#     ${Boost_INCLUDE_DIR}
#     ${LIBELAS_INCLUDE_DIRS}
#     /opt/ros/hydro/include
)

target_link_libraries(rsgm_ros
#   ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
# #   ${CUDA_LIBRARIES}
#   ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)