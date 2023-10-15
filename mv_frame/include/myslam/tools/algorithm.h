//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "myslam/common_include.h"

namespace myslam
{

    /**
     * linear triangulation with SVD
     * @param poses     poses,
     * @param points    points in normalized plane
     * @param pt_world  triangulated point in the world
     * @return true if success
     */
    inline bool triangulation(const std::vector<SE3> &poses,
                              const std::vector<Vec3> points, Vec3 &pt_world)
    {
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i)
        {
            Mat34 m = poses[i].matrix3x4();
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
        {
            // 解质量不好，放弃
            return true;
        }
        return false;
    }

    // converters
    inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

    // TODO暂时使用，后面可改成用DetectFeatures()
    inline void featureTracking(Mat img_1, Mat img_2, vector<Point2f> &points1, vector<Point2f> &points2, vector<uchar> &status)
    {

        // this function automatically gets rid of points for which tracking fails

        vector<float> err;
        Size winSize = Size(21, 21);
        TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

        calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

        // getting rid of points for which the KLT tracking failed or those who have gone outside the frame
        int indexCorrection = 0;
        for (int i = 0; i < status.size(); i++)
        {
            Point2f pt = points2.at(i - indexCorrection);
            if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
            {
                if ((pt.x < 0) || (pt.y < 0))
                {
                    status.at(i) = 0;
                }
                points1.erase(points1.begin() + (i - indexCorrection));
                points2.erase(points2.begin() + (i - indexCorrection));
                indexCorrection++;
            }
        }
    }

    inline void featureDetection(Mat img_1, vector<Point2f> &points1)
    { // uses FAST as of now, modify parameters as necessary
        vector<KeyPoint> keypoints_1;
        int fast_threshold = 20;
        bool nonmaxSuppression = true;
        FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
        KeyPoint::convert(keypoints_1, points1, vector<int>());
    }

    inline SE3 Rt2T(Mat R, Mat t)
    {
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_t;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                eigen_R(i, j) = R.at<double>(i, j);
            }
        }
        for (int i = 0; i < 3; i++)
        {
            eigen_t(i) = t.at<double>(i);
        }

        // 创建Sophus::SE3d类型的变换矩阵T
        Sophus::SE3d T(eigen_R, eigen_t);
        return T;
    }

} // namespace myslam

#endif // MYSLAM_ALGORITHM_H
