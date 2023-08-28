#include "myslam/visual_odometry.h"
#include <string>
int main(int argc, char **argv)
{

  std::string config_path = "/home/alpaca/v_slam_lesson/my_slam_ws/mv_frame/config/default.yaml";
  myslam::VisualOdometry::Ptr vo(
      new myslam::VisualOdometry(config_path, myslam::VisualOdometry::RGBD));
  assert(vo->Init() == true);
  vo->Run();

  return 0;
}