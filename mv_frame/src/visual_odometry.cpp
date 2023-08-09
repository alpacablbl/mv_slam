//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config/config.h"

namespace myslam
{

    VisualOdometry::VisualOdometry(std::string &config_path, const eSensor sensor)
        : config_file_path_(config_path), sensor_(sensor) 
        {
            // 输出当前传感器类型
            cout << "Input sensor was set to: ";

            if(sensor_==MONOCULAR)
                cout << "Monocular" << endl;
            else if(sensor_==STEREO)
                cout << "Stereo" << endl;
            else if(sensor_==RGBD)
                cout << "RGB-D" << endl;
        }

    bool VisualOdometry::Init()
    {
        // read from config file
        if (Config::SetParameterFile(config_file_path_) == false)
        {
            return false;
        }

        dataset_ =
            Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        CHECK_EQ(dataset_->Init(), true);

        // create components and links
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1)); // 取左右灰度相机

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        viewer_->SetMap(map_);

        return true;
    }

    void VisualOdometry::Run()
    {
        while (1)
        {
            LOG(INFO) << "VO is running";
            if (Step() == false)
            {
                break;
            }
        }

        backend_->Stop();
        viewer_->Close();

        LOG(INFO) << "VO exit";
    }

    // 读不出数据就会停止
    bool VisualOdometry::Step()
    {
        Frame::Ptr new_frame = dataset_->NextFrame();
        if (new_frame == nullptr)
            return false;

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }

} // namespace myslam
