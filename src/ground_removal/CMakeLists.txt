add_library(depth_ground_remover SHARED depth_ground_remover.cpp)
target_link_libraries(depth_ground_remover
                      identifiable
                      velodyne_utils
                      difference
                      savitsky_golay_smoothing
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
            roscpp
            visualization_msgs
            )
add_library(tunnel_ground_remover SHARED tunnel_ground_remover.cpp)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
#!!!

target_link_libraries(tunnel_ground_remover
                      identifiable
                      velodyne_utils
                      difference
                      savitsky_golay_smoothing
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})