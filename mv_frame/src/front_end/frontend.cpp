//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/opencv.hpp>

#include "myslam/visual_odometry.h"
#include "myslam/tools/algorithm.h"
#include "myslam/back_end/backend.h"
#include "myslam/config/config.h"
#include "myslam/feature/feature.h"
#include "myslam/front_end/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map/map.h"
#include "myslam/viewer/viewer.h"

#define MIN_NUM_FEAT 2000

namespace myslam
{

    Frontend::Frontend(const int sensor) : fsensor_(sensor)
    {
        gftt_ =
            cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }

    Frontend::Frontend(const int sensor, Frame::Ptr mono_frame) : fsensor_(sensor), fmono_frame_(mono_frame)
    {
        gftt_ =
            cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }

    bool Frontend::AddFrame(myslam::Frame::Ptr frame)
    {
        current_frame_ = frame;

        switch (status_)
        {
        case FrontendStatus::INITING:
            if (fsensor_ == VisualOdometry::STEREO)
                StereoInit();
            else if (fsensor_ == VisualOdometry::RGBD)
                RgbdInit();
            else if (fsensor_ == VisualOdometry::MONOCULAR)
                MonoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            if (fsensor_ == VisualOdometry::MONOCULAR)
                monoTrack();
            else
                Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
        }

        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::monoTrack()
    {
        // int num_track_last = TrackLastFrame();

        // mono不需要icp估计位姿
        // TODO mono自己的位姿计算函数
        if (last_frame_->mono_Features_.size() < MIN_NUM_FEAT)
        {
            DetectFeatures(last_frame_);
        }
        CalMonoCurrentPose();

        tracking_inliers_ = EstimateCurrentPose();

        if (tracking_inliers_ > num_features_tracking_)
        {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > num_features_tracking_bad_)
        {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else
        {
            // lost
            status_ = FrontendStatus::LOST;
        }

        // TODO 对于mono，需要多detectFeature一次
        InsertKeyframe();

        if (viewer_)
            viewer_->AddCurrentFrame(current_frame_);
        return true;
    }

    bool Frontend::Track()
    {
        if (last_frame_)
        {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }

        int num_track_last = TrackLastFrame();

        // mono不需要icp估计位姿
        tracking_inliers_ = EstimateCurrentPose();

        if (tracking_inliers_ > num_features_tracking_)
        {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > num_features_tracking_bad_)
        {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else
        {
            // lost
            status_ = FrontendStatus::LOST;
        }

        InsertKeyframe();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        if (viewer_)
            viewer_->AddCurrentFrame(current_frame_);
        return true;
    }

    bool Frontend::InsertKeyframe()
    {
        if (tracking_inliers_ >= num_features_needed_for_keyframe_)
        {
            // still have enough features, don't insert keyframe
            return false;
        }
        // current frame is a new keyframe
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;

        SetObservationsForKeyFrame();
        // TODO stereo and rgbd 需要检测特征，mono在monotrack中已经做了lastframe的特征检测
        if (fsensor_ != VisualOdometry::MONOCULAR)
        {
            DetectFeatures(); // detect new features
        }

        if (fsensor_ == VisualOdometry::STEREO)
        {
            // track in right image
            FindFeaturesInRight();
            // triangulate map points
            TriangulateNewPoints();
        }
        else if (fsensor_ == VisualOdometry::RGBD)
        {
            // TODO
            RgbdAddMappoint();
        }
        else if (fsensor_ == VisualOdometry::MONOCULAR)
        {
            // TODO track in last image
            FindFeaturesInCurrent();
            // triangulate map points
            TriangulateMonoNewPoints();
        }

        // update backend because we have a new keyframe
        backend_->UpdateMap();

        if (viewer_)
            viewer_->UpdateMap();

        return true;
    }

    void Frontend::SetObservationsForKeyFrame()
    {
        for (auto &feat : current_frame_->features_left_)
        {
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->AddObservation(feat);
        }
    }

    int Frontend::TriangulateNewPoints()
    {
        // 两个位置的相机位姿
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr)
            {
                // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->position_.pt.x,
                             current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                        current_frame_->features_left_[i]);
                    new_map_point->AddObservation(
                        current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    int Frontend::TriangulateMonoNewPoints()
    {
        // TODO 换成cur和ref frame的pose
        std::vector<SE3> poses{last_frame_->Pose(), current_frame_->Pose()};
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < last_frame_->features_left_.size(); ++i)
        {
            if (last_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_left_[i] != nullptr)
            {
                // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(last_frame_->features_left_[i]->position_.pt.x,
                             last_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(last_frame_->features_left_[i]);
                    new_map_point->AddObservation(current_frame_->features_left_[i]);

                    last_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    // TODO
    void Frontend::RgbdAddMappoint()
    {
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i]->map_point_.expired())
            {
                // TODO 换算深度后映射成3D点加入到地图中
                int x = current_frame_->features_left_[i]->position_.pt.x;
                int y = current_frame_->features_left_[i]->position_.pt.y;
                Vec2 temppix = {x, y};
                x = cvRound(x);
                y = cvRound(y);
                double dd = 0;
                ushort d = current_frame_->depth_.ptr<ushort>(y)[x];

                if (d != 0)
                {
                    dd = double(d) / 5000.0;
                }
                else
                {
                    // check the nearby points
                    int dx[4] = {-1, 0, 1, 0};
                    int dy[4] = {0, -1, 0, 1};
                    for (int i = 0; i < 4; i++)
                    {
                        d = current_frame_->depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
                        if (d != 0)
                        {
                            dd = double(d) / 5000.0;
                        }
                    }
                }
                Vec3 p1 = camera_left_->pixel2camera(temppix);
                Vec3 pworld = dd * p1;

                if (dd > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                        current_frame_->features_left_[i]);
                    // new_map_point->AddObservation(
                    //     current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    // current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    }

    void Frontend::CalMonoCurrentPose()
    {
        vector<uchar> status;
        vector<Point2f> points2;
        // 由pic1和其特征点，通过光流计算出points2(对应在pic2上的特征点)
        featureTracking(last_frame_->left_img_, current_frame_->left_img_, last_frame_->mono_Features_, points2, status);
        Mat E, R, t, initK, mask;
        initK = Frame::initK_;
        E = findEssentialMat(points2, last_frame_->mono_Features_, initK, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, points2, last_frame_->mono_Features_, initK, R, t, mask);

        // 创建Sophus::SE3d类型的变换矩阵T
        SE3 T = Rt2T(R, t);
        current_frame_->SetPose(T);
    }

    // 估计当前位姿并计算内点
    int Frontend::EstimateCurrentPose()
    {
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
            LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // vertex
        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        // K
        Mat33 K = camera_left_->K();

        // edges
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            if (mp)
            {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                    toVec2(current_frame_->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        // estimate the Pose the determine the outliers
        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration)
        {
            vertex_pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if (features[i]->is_outlier_)
                {
                    e->computeError();
                }
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        // LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
        //           << features.size() - cnt_outlier;
        // Set pose and outlier

        // 非mono用对极几何求位姿
        if (fsensor_ != VisualOdometry::MONOCULAR)
        {
            current_frame_->SetPose(vertex_pose->estimate());
        }

        // LOG(INFO) << "Current Pose = \n"
        //           << current_frame_->Pose().matrix();

        for (auto &feat : features)
        {
            if (feat->is_outlier_)
            {
                feat->map_point_.reset();
                feat->is_outlier_ = false; // maybe we can still use it in future
            }
        }
        return features.size() - cnt_outlier;
    }

    // 返回好点数目？
    int Frontend::TrackLastFrame()
    {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_)
        {
            if (kp->map_point_.lock())
            {
                // use project point
                auto mp = kp->map_point_.lock();
                auto px =
                    camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, kps_last,
            kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }

    bool Frontend::StereoInit()
    {
        int num_features_left = DetectFeatures();
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init_)
        {
            return false;
        }

        bool build_map_success = BuildInitMap();
        if (build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_)
            {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }
        return false;
    }

    bool Frontend::RgbdInit()
    {
        int num_features_left = DetectFeatures();

        bool build_map_success = BuildInitRgbdMap();
        if (build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_)
            {

                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }
        return false;
    }

    // TODO 特征点检测，初始化地图都要修改
    bool Frontend::MonoInit()
    {

        //===================================================================//
        // int num_features_left = DetectFeatures();
        DetectFeatures(fmono_frame_);
        last_frame_ = fmono_frame_;

        CalMonoCurrentPose();
        FindFeaturesInCurrent();

        bool build_map_success = BuildInitMonoMap();
        if (build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_)
            {
                // TODO 这俩可能也要单独改
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }
        return false;
    }

    int Frontend::DetectFeatures()
    {
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_)
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int Frontend::DetectFeatures(Frame::Ptr t_frame)
    {
        std::vector<KeyPoint> keypoints_1;
        std::vector<cv::KeyPoint> keypoints;
        int fast_threshold = 20;
        bool nonmaxSuppression = true;
        // 顺序会对结果有影响吗？
        FAST(t_frame->left_img_, keypoints_1, fast_threshold, nonmaxSuppression);
        KeyPoint::convert(keypoints_1, t_frame->mono_Features_, vector<int>());

        int cnt_detected = 0;
        for (auto &kp : keypoints_1)
        {
            t_frame->features_left_.push_back(
                Feature::Ptr(new Feature(t_frame, kp)));
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight()
    {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->map_point_.lock();
            // 若对象存在，则指向这个weakptr对应的sharedptr指向的对象
            if (mp)
            {
                // use projected points as initial guess
                auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                // use same pixel in left iamge
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
        }
        LOG(INFO) << "Find " << num_good_pts << " in the right image.";
        return num_good_pts;
    }

    // TODO mono use
    // 通过lastframe算出currentframe的features，存储在features_left_和mono_Features_[Point2f]两种类型的数据结构里
    int Frontend::FindFeaturesInCurrent()
    {
        // use LK flow to estimate points in the current image

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, last_frame_->mono_Features_,
            current_frame_->mono_Features_, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(current_frame_->mono_Features_[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                // feat->is_on_left_image_ = false;
                current_frame_->features_left_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->features_left_.push_back(nullptr);
            }
        }
        LOG(INFO) << "Find " << num_good_pts << " in the current image.";
        return num_good_pts;
    }

    // TODO RGBD相机初始化不用三角化，直接判断一下深度是否合法就可以加进去了
    bool Frontend::BuildInitMap()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_right_[i] == nullptr)
                continue;
            // create map point from triangulation
            // 根据左右相机位姿和某一匹配特征点，计算出此特征点的深度
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            // 若通过三角测距成功计算出深度
            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);                                     // world系下3d点加入地图点中
                new_map_point->AddObservation(current_frame_->features_left_[i]);  // 可删
                new_map_point->AddObservation(current_frame_->features_right_[i]); // 可删
                current_frame_->features_left_[i]->map_point_ = new_map_point;     // 2d特征点在三角化之后会被关联一个地图点
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;                /// 路标数量增加
                map_->InsertMapPoint(new_map_point); // 初始化完成的地图点加入地图
            }
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap(); // 关键帧的插入会启动优化

        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }

    // TODO
    bool Frontend::BuildInitMonoMap()
    {
        std::vector<SE3> poses{last_frame_->Pose(), current_frame_->Pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < last_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i] == nullptr)
                continue;
            // create map point from triangulation
            // 根据左右相机位姿和某一匹配特征点，计算出此特征点的深度
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(last_frame_->features_left_[i]->position_.pt.x,
                         last_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            // 若通过三角测距成功计算出深度
            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);                                    // world系下3d点加入地图点中
                new_map_point->AddObservation(last_frame_->features_left_[i]);    // 可删
                new_map_point->AddObservation(current_frame_->features_left_[i]); // 可删
                last_frame_->features_left_[i]->map_point_ = new_map_point;       // 2d特征点在三角化之后会被关联一个地图点
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;                /// 路标数量增加
                map_->InsertMapPoint(new_map_point); // 初始化完成的地图点加入地图
            }
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap(); // 关键帧的插入会启动优化

        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }

    bool Frontend::BuildInitRgbdMap()
    {

        int cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {

            // TODO depth要改成depth_ Mat类型
            int x = current_frame_->features_left_[i]->position_.pt.x;
            int y = current_frame_->features_left_[i]->position_.pt.y;
            Vec2 temppix = {x, y};
            x = cvRound(x);
            y = cvRound(y);
            double dd = 0;
            ushort d = current_frame_->depth_.ptr<ushort>(y)[x];

            if (d != 0)
            {
                dd = double(d) / 5000.0;
            }
            else
            {
                // check the nearby points
                int dx[4] = {-1, 0, 1, 0};
                int dy[4] = {0, -1, 0, 1};
                for (int i = 0; i < 4; i++)
                {
                    d = current_frame_->depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
                    if (d != 0)
                    {
                        dd = double(d) / 5000.0;
                    }
                }
            }
            Vec3 p1 = camera_left_->pixel2camera(temppix);
            Vec3 pworld = dd * p1;

            if (dd > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();

                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                // new_map_point->AddObservation(
                //     current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                // current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_init_landmarks++;
            }
        }

        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap(); // 关键帧的插入会启动优化

        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }

    bool Frontend::Reset()
    {
        LOG(INFO) << "Reset is not implemented. ";
        return true;
    }

} // namespace myslam