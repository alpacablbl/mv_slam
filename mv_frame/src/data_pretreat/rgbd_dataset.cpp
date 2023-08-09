#include "myslam/data_pretreat/dataset.h"
#include "myslam/map/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam
{

    RgbdDataset::RgbdDataset(const std::string &dataset_path)
        : dataset_path_(dataset_path) {}

    bool RgbdDataset::Init()
    {
        // read camera intrinsics and extrinsics
        ifstream fin(dataset_path_ + "/calib.txt");
        if (!fin)
        {
            LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
            return false;
        }

        
        //目前直接硬编码相机内参
        Mat33 K;
        K << 517.3, 0, 325.1,
            0, 516.5, 249.7,
            0, 0, 1;
        Vec3 t;

        //TODO，需要生成虚拟右目 得到baseline和t
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                            t.norm(), SE3(SO3(), t))); // 初始化时仅有几个相机间的平移关系

        //matK与distCoeffs用来去畸变                                    
        new_camera.matK = (cv::Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1);
        new_camera.distCoeffs = cv::Mat::zeros(5,1,CV_64F);
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();

        fin.close();
        current_image_index_ = 0;
        return true;
    }

    // 读入下一帧
    Frame::Ptr RgbdDataset::NextFrame()
    {
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat image_left, image_right;
        // read images
        image_left =
            cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                       cv::IMREAD_GRAYSCALE);
        image_right =
            cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                       cv::IMREAD_GRAYSCALE);

        if (image_left.data == nullptr || image_right.data == nullptr)
        {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }

        cv::Mat image_left_resized, image_right_resized;
        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);

        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left_resized;
        new_frame->right_img_ = image_right_resized;
        current_image_index_++;
        return new_frame;
    }

} // namespace myslam