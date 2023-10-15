#ifndef MYSLAM_MONO_DATASET_H
#define MYSLAM_MONO_DATASET_H
#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/map/frame.h"

using namespace cv;
using namespace std;

namespace myslam
{

  /**
   * 数据集读取
   * 构造时传入配置文件路径，配置文件的dataset_dir为数据集路径
   * Init之后可获得相机和下一帧图像
   */
  class MonoDataset
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MonoDataset> Ptr;
    MonoDataset(const std::string &dataset_path);

    /// 初始化，返回是否成功
    bool Init();

    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    Frame::Ptr getMonoInitFrame()
    {
      return momo_init_frame_;
    }

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const
    {
      return cameras_.at(camera_id);
    }

  private:
    std::string dataset_path_;
    int current_image_index_ = 0;
    // 通过前两帧进行初始化后得到的位姿
    Mat R_, t_;
    Frame::Ptr momo_init_frame_ = nullptr;
    bool ismonofirst_scan_ = false;

    std::vector<Camera::Ptr> cameras_;
  };
} // namespace myslam

#endif