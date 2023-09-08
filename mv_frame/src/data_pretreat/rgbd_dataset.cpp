#include "myslam/data_pretreat/rgbd_dataset.h"
#include "myslam/map/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam
{

    RgbdDataset::RgbdDataset(const std::string &dataset_path)
        : dataset_path_(dataset_path), grbd_frame_index_(0) {}

    bool RgbdDataset::Init()
    {
        // read camera intrinsics and extrinsics
        ifstream fin(dataset_path_ + "/associations.txt");
        if (!fin)
        {
            LOG(ERROR) << "cannot find " << dataset_path_ << "/associations.txt";
            return false;
        }

        // 将对齐的rgb图，深度图和时间戳存入vector
        std::string assPath = dataset_path_ + "/associations.txt";
        std::ifstream txt;
        txt.open(assPath.data());
        assert(txt.is_open());
        while (!txt.eof())
        {
            string rgb_time, rgb_file, depth_time, depth_file;
            txt >> rgb_time >> rgb_file >> depth_time >> depth_file;
            rgb_times_.push_back(atof(rgb_time.c_str()));
            depth_times_.push_back(atof(depth_time.c_str()));
            rgb_files_.push_back(dataset_path_ + "/" + rgb_file);
            depth_files_.push_back(dataset_path_ + "/" + depth_file);

            if (txt.good() == false)
                break;
        }

        // 目前直接硬编码相机内参
        Mat33 K;
        K << 517.3, 0, 325.1,
            0, 516.5, 249.7,
            0, 0, 1;
        Vec3 t;

        // TODO，需要生成虚拟右目 得到baseline和t
        t << 0, 0, 0;
        t = K.inverse() * t;
        K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t))); // 初始化时仅有几个相机间的平移关系

        // matK与distCoeffs用来去畸变
        new_camera->matK = (cv::Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1);

        new_camera->distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        cameras_.push_back(new_camera);

        fin.close();
        current_image_index_ = 0;
        return true;
    }

    // 读入下一帧
    Frame::Ptr RgbdDataset::NextFrame()
    {

        Mat image_left = cv::imread(rgb_files_[grbd_frame_index_], cv::IMREAD_GRAYSCALE);
        Mat image_depth = cv::imread(depth_files_[grbd_frame_index_], cv::IMREAD_UNCHANGED);

        if (grbd_frame_index_ >= rgb_files_.size())
        {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }

        // cv::Mat image_left_resized;
        // cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
        //            cv::INTER_NEAREST);

        auto new_frame = Frame::CreateFrame();
        // TODO depth不做resize会有问题吗？
        new_frame->left_img_ = image_left;
        new_frame->depth_ = image_depth;
        current_image_index_++;
        grbd_frame_index_++;

        return new_frame;
    }

} // namespace myslam