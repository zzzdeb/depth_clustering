add_library(ros_bridge SHARED cloud_odom_ros_subscriber.cpp 
                                laser_ros_subscriber.cpp 
                                )
target_link_libraries(ros_bridge
                      identifiable
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})

add_library(ros_vis SHARED ros_visualizer.cpp)                              

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  OpenCV
)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(ros_vis
                      identifiable
                      visualization
                      ${OpenCV_LIBRARIES}
                      ${catkin_LIBRARIES}
                    #   ${catkin_REQUIRED}
                      ${QGLVIEWER_LIBRARY}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})