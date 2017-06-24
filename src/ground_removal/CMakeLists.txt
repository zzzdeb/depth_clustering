add_library(ground_remove SHARED depth_ground_remover.cpp)
target_link_libraries(ground_remove
                      identifiable
                      velodyne_utils
                      difference
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})

find_package(PCL 1.7 REQUIRED)
add_library(tunnel_ground_removal SHARED tunnel_ground_remover.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(tunnel_ground_removal
                      identifiable
                      velodyne_utils
                      difference
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})