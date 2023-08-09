#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/back_end/backend.h"
#include "myslam/common_include.h"
#include "myslam/data_pretreat/dataset.h"
#include "myslam/front_end/frontend.h"
#include "myslam/viewer/viewer.h"

namespace myslam
{

    /**
     * VO 对外接口
     */
    class VisualOdometry
    {
    public:
            enum eSensor{
            MONOCULAR=0,
            STEREO=1,
            RGBD=2
        };
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        /// constructor with config file
        VisualOdometry(std::string &config_path, const eSensor sensor);

        /**
         * do initialization things before run
         * @return true if success
         */
        bool Init();

        /**
         * start vo in the dataset
         */
        void Run();

        /**
         * Make a step forward in dataset
         */
        bool Step();

        /// 获取前端状态
        FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

    private:
        eSensor sensor_;

        bool inited_ = false;
        std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

        // dataset
        Dataset::Ptr dataset_ = nullptr;
    };
} // namespace myslam

#endif // MYSLAM_VISUAL_ODOMETRY_H
