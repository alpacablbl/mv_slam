add_executable(run_kitti_stereo run_kitti_stereo.cpp)
target_link_libraries(run_kitti_stereo myslam ${THIRD_PARTY_LIBS})
add_executable(run_rgbd_tum run_rgbd_tum.cpp)
target_link_libraries(run_rgbd_tum myslam ${THIRD_PARTY_LIBS})