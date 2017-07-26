add_library(cloud SHARED cloud.cpp rich_point.cpp)
target_link_libraries(cloud
                      projections
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})

add_library(velodyne_utils SHARED velodyne_utils.cpp)
target_link_libraries(velodyne_utils
                      projections
                      cloud
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})

find_package(Boost REQUIRED COMPONENTS filesystem regex)
add_library(folder_reader SHARED folder_reader.cpp)
target_link_libraries(folder_reader
                      ${Boost_LIBRARIES})

add_library(savitsky_golay_smoothing SHARED savitsky_golay_smoothing.cpp)
target_link_libraries(savitsky_golay_smoothing
                      identifiable
                      velodyne_utils
                      difference
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})

add_library(oriented_bounding_box SHARED oriented_bounding_box.cpp)
target_link_libraries(oriented_bounding_box
                        ${PCL_LIBRARIES})          