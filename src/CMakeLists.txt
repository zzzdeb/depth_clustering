add_subdirectory(communication)
add_subdirectory(ground_removal)
add_subdirectory(image_labelers)
add_subdirectory(projections)
add_subdirectory(utils)
add_subdirectory(ros_bridge)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            )

add_executable(depth_clusterer depth_clusterer.cpp)
target_link_libraries(depth_clusterer
                        identifiable
                        image_labeler
                        velodyne_utils
                        projections
                        depth_ground_remover
                        tunnel_ground_remover
                        folder_reader
                        ros_visualizer
                        ros_bridge
                        ${catkin_LIBRARIES})
