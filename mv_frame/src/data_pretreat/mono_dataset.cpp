#include "myslam/data_pretreat/mono_dataset.h"
#include "myslam/map/frame.h"
#include "myslam/tools/algorithm.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam
{
  Mat Frame::initK_ = cv::Mat_<double>(3, 3);
  MonoDataset::MonoDataset(const std::string &dataset_path)
      : dataset_path_(dataset_path) {}

  bool MonoDataset::Init()
  {
    // read camera intrinsics and extrinsics
    ifstream fin(dataset_path_ + "/calib.txt");
    Mat initK;
    if (!fin)
    {
      LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
      return false;
    }

    for (int i = 0; i < 1; ++i)
    {
      char camera_name[3]; //"P0"两个字符，开3个char，4个相机：左灰， 右灰，左彩，右彩
      for (int k = 0; k < 3; ++k)
      {
        fin >> camera_name[k];
      }
      double projection_data[12]; // 内参矩阵
      for (int k = 0; k < 12; ++k)
      {
        fin >> projection_data[k];
      }
      Mat33 K;
      K << projection_data[0], projection_data[1], projection_data[2],
          projection_data[4], projection_data[5], projection_data[6],
          projection_data[8], projection_data[9], projection_data[10];
      initK = cv::Mat_<double>(3, 3) << projection_data[0], projection_data[1], projection_data[2],
      projection_data[4], projection_data[5], projection_data[6],
      projection_data[8], projection_data[9], projection_data[10];
      Frame::initK_ = initK;
      Vec3 t;
      t << projection_data[3], projection_data[7], projection_data[11];
      t = K.inverse() * t;
      K = K * 0.5;
      Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                        t.norm(), SE3(SO3(), t))); // 初始化时仅有几个相机间的平移关系
      cameras_.push_back(new_camera);
      LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }

    fin.close();

    // TODO尚未对1，2进行有效性判断【后期加上】
    Mat img_1, img_2;
    string filename1 = dataset_path_ + "image_0/0.png";
    string filename2 = dataset_path_ + "image_0/1.png";
    Mat img_1_c = imread(filename1);
    Mat img_2_c = imread(filename2);

    if (!img_1_c.data || !img_2_c.data)
    {
      std::cout << " --(!) Error reading images " << std::endl;
      return -1;
    }
    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);
    vector<Point2f> points1, points2;
    featureDetection(img_1, points1);
    vector<uchar> status;
    // 由pic1和其特征点，通过光流计算出points2(对应在pic2上的特征点)
    featureTracking(img_1, img_2, points1, points2, status);
    Mat E, mask;
    E = findEssentialMat(points2, points1, initK, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1, initK, R_, t_, mask);

    // TODO 把以上信息打包成frame 创建新成员变量 后面传给front
    momo_init_frame_ = Frame::CreateFrame();
    cv::Mat image2_left_resized;
    cv::resize(img_2, image2_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    momo_init_frame_->left_img_ = image2_left_resized;

    // 创建Sophus::SE3d类型的变换矩阵T
    Sophus::SE3d T = Rt2T(R_, t_);
    momo_init_frame_->pose_ = T;

    current_image_index_ = 2; // 0, 1被用来初始化了，从2开始
    return true;
  }

  // 读入下一帧
  Frame::Ptr MonoDataset::NextFrame()
  {
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left;
    // read images
    image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);

    if (image_left.data == nullptr)
    {
      LOG(WARNING) << "cannot find images at index " << current_image_index_;
      return nullptr;
    }

    cv::Mat image_left_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left_resized;
    current_image_index_++;

    // //=========TODO判断是否第一帧==========//
    // if (ismonofirst_scan_ == false)
    // {
    //   // 用两帧开始初始化

    //   ismonofirst_scan_ == true;
    // }

    return new_frame;
  }

} // namespace myslam