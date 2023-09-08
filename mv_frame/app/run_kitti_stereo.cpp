
#include "myslam/visual_odometry.h"
#include <string>
int main(int argc, char **argv)
{

    std::string config_path = "/home/alpaca/mv_slam/mv_frame/config/default.yaml";
    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(config_path, myslam::VisualOdometry::STEREO));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
