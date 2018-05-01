add_executable(speed_test speed_test.cpp)                          

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
)

target_link_libraries(speed_test
                      identifiable
                      projections
                      velodyne_utils
                      folder_reader
                      savitsky_golay_smoothing
                      depth_ground_remover
                      ${catkin_LIBRARIES})

add_executable(topic_hz_listener topic_hz_listener.cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

target_link_libraries(topic_hz_listener
              ${catkin_LIBRARIES}          
)       