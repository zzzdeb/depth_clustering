add_library(ros_bridge SHARED cloud_odom_ros_subscriber.cpp 
                                image_ros_subscriber.cpp 
                                )
target_link_libraries(ros_bridge
                      identifiable
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})


find_package(catkin REQUIRED COMPONENTS
                      roscpp
                      sensor_msgs
                      image_transport
                      cv_bridge
                    )
add_executable(image_publisher depth_image_publisher.cpp 
                      )
target_link_libraries(image_publisher
            identifiable
            folder_reader
            velodyne_utils
            ${catkin_LIBRARIES}
            )

find_package(PCL 1.7 REQUIRED)

add_library(objects_publisher SHARED objects_publisher.cpp)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(objects_publisher
                      identifiable
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})


add_library(ros_visualizer SHARED ros_visualizer.cpp)                              

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  OpenCV
)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(ros_visualizer
                      identifiable
                      objects_publisher
                      ${OpenCV_LIBRARIES}
                      ${catkin_LIBRARIES}
                    #   ${catkin_REQUIRED}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})