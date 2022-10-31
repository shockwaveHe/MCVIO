#include "solve_5pts.h"
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace cv
{
    void
    decomposeEssentialMat(InputArray _E,
                          OutputArray _R1,
                          OutputArray _R2,
                          OutputArray _t)
    {

        Mat E = _E.getMat().reshape(1, 3);
        CV_Assert(E.cols == 3 && E.rows == 3);

        Mat D, U, Vt;
        SVD::compute(E, D, U, Vt);

        if (determinant(U) < 0)
            U *= -1.;
        if (determinant(Vt) < 0)
            Vt *= -1.;

        Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
        W.convertTo(W, E.type());

        Mat R1, R2, t;
        R1 = U * W * Vt;
        R2 = U * W.t() * Vt;
        t = U.col(2) * 1.0;

        R1.copyTo(_R1);
        R2.copyTo(_R2);
        t.copyTo(_t);
    }

    int
    recoverPose(InputArray E,
                InputArray _points1, InputArray _points2,
                InputArray _cameraMatrix,
                OutputArray _R, OutputArray _t,
                InputOutputArray _mask)
    {

        Mat points1, points2, cameraMatrix;
        _points1.getMat().convertTo(points1, CV_64F);
        _points2.getMat().convertTo(points2, CV_64F);
        _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

        int npoints = points1.checkVector(2);
        CV_Assert(npoints >= 0 && points2.checkVector(2) == npoints &&
                  points1.type() == points2.type());

        CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

        if (points1.channels() > 1)
        {
            points1 = points1.reshape(1, npoints);
            points2 = points2.reshape(1, npoints);
        }

        double fx = cameraMatrix.at<double>(0, 0);
        double fy = cameraMatrix.at<double>(1, 1);
        double cx = cameraMatrix.at<double>(0, 2);
        double cy = cameraMatrix.at<double>(1, 2);

        points1.col(0) = (points1.col(0) - cx) / fx;
        points2.col(0) = (points2.col(0) - cx) / fx;
        points1.col(1) = (points1.col(1) - cy) / fy;
        points2.col(1) = (points2.col(1) - cy) / fy;

        points1 = points1.t();
        points2 = points2.t();

        Mat R1, R2, t;
        decomposeEssentialMat(E, R1, R2, t);
        Mat P0 = Mat::eye(3, 4, R1.type());
        Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
        P1(Range::all(), Range(0, 3)) = R1 * 1.0;
        P1.col(3) = t * 1.0;
        P2(Range::all(), Range(0, 3)) = R2 * 1.0;
        P2.col(3) = t * 1.0;
        P3(Range::all(), Range(0, 3)) = R1 * 1.0;
        P3.col(3) = -t * 1.0;
        P4(Range::all(), Range(0, 3)) = R2 * 1.0;
        P4.col(3) = -t * 1.0;

        // Do the cheirality check.
        // Notice here a threshold dist is used to filter
        // out far away points (i.e. infinite points) since
        // there depth may vary between postive and negtive.
        double dist = 50.0;
        Mat Q;
        triangulatePoints(P0, P1, points1, points2, Q);
        Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask1 = (Q.row(2) < dist) & mask1;
        Q = P1 * Q;
        mask1 = (Q.row(2) > 0) & mask1;
        mask1 = (Q.row(2) < dist) & mask1;

        triangulatePoints(P0, P2, points1, points2, Q);
        Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask2 = (Q.row(2) < dist) & mask2;
        Q = P2 * Q;
        mask2 = (Q.row(2) > 0) & mask2;
        mask2 = (Q.row(2) < dist) & mask2;

        triangulatePoints(P0, P3, points1, points2, Q);
        Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask3 = (Q.row(2) < dist) & mask3;
        Q = P3 * Q;
        mask3 = (Q.row(2) > 0) & mask3;
        mask3 = (Q.row(2) < dist) & mask3;

        triangulatePoints(P0, P4, points1, points2, Q);
        Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask4 = (Q.row(2) < dist) & mask4;
        Q = P4 * Q;
        mask4 = (Q.row(2) > 0) & mask4;
        mask4 = (Q.row(2) < dist) & mask4;

        mask1 = mask1.t();
        mask2 = mask2.t();
        mask3 = mask3.t();
        mask4 = mask4.t();

        // If _mask is given, then use it to filter outliers.
        if (!_mask.empty())
        {
            Mat mask = _mask.getMat();
            CV_Assert(mask.size() == mask1.size());
            bitwise_and(mask, mask1, mask1);
            bitwise_and(mask, mask2, mask2);
            bitwise_and(mask, mask3, mask3);
            bitwise_and(mask, mask4, mask4);
        }
        if (_mask.empty() && _mask.needed())
        {
            _mask.create(mask1.size(), CV_8U);
        }

        CV_Assert(_R.needed() && _t.needed());
        _R.create(3, 3, R1.type());
        _t.create(3, 1, t.type());

        int good1 = countNonZero(mask1);
        int good2 = countNonZero(mask2);
        int good3 = countNonZero(mask3);
        int good4 = countNonZero(mask4);

        if (good1 >= good2 && good1 >= good3 && good1 >= good4)
        {
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask1.copyTo(_mask);
            return good1;
        }
        else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
        {
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask2.copyTo(_mask);
            return good2;
        }
        else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
        {
            t = -t;
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask3.copyTo(_mask);
            return good3;
        }
        else
        {
            t = -t;
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask4.copyTo(_mask);
            return good4;
        }
    }

    int
    recoverPose(InputArray E,
                InputArray _points1, InputArray _points2,
                OutputArray _R, OutputArray _t,
                double focal, Point2d pp,
                InputOutputArray _mask)
    {
        Mat cameraMatrix = (Mat_<double>(3, 3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);
        return cv::recoverPose(E, _points1, _points2, cameraMatrix, _R, _t, _mask);
    }
}

bool MotionEstimator::solveRelativeRT(const Eigen::aligned_vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &corres,
                                      Eigen::Matrix3d &Rotation,
                                      Eigen::Vector3d &Translation)
{
    if (corres.size() >= 15)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        cv::Mat mask;
        cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 1490, 0.99, mask);
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat rot, trans;
        int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
        // cout << "inlier_cnt " << inlier_cnt << endl;

        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        for (int i = 0; i < 3; i++)
        {
            T(i) = trans.at<double>(i, 0);
            for (int j = 0; j < 3; j++)
                R(i, j) = rot.at<double>(i, j);
        }

        Rotation = R.transpose();
        Translation = -R.transpose() * T;
        if (inlier_cnt > 12)
        {
            ROS_ERROR_STREAM("----------5points-----------");
            ROS_ERROR_STREAM(ll.size());
            ROS_ERROR_STREAM(inlier_cnt);
            ROS_ERROR_STREAM(Rotation);
            ROS_ERROR_STREAM(Translation);
            return true;
        }
        else
            return false;
    }
    return false;
}

bool MotionEstimator::solveRelativeRT_PNP(const Eigen::aligned_vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &corres,
                                          Eigen::Matrix3d &Rotation,
                                          Eigen::Vector3d &Translation)
{

    vector<cv::Point3f> lll;
    vector<cv::Point2f> rr;
    for (int i = 0; i < int(corres.size()); i++)
    {
        if (corres[i].first(2) > 0 && corres[i].second(2) > 0)
        {
            lll.push_back(cv::Point3f(corres[i].first(0), corres[i].first(1), corres[i].first(2)));
            rr.push_back(cv::Point2f(corres[i].second(0) / corres[i].second(2),
                                     corres[i].second(1) / corres[i].second(2)));
        }
    }
    cv::Mat rvec, tvec, inliersArr;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    cv::solvePnPRansac(lll, rr, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0 / 460, 0.99,
                       inliersArr, cv::SOLVEPNP_ITERATIVE); // maybe don't need 100times

    Eigen::Vector3d tran(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));

    Eigen::Vector3d omega;
    omega << tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0);
    Eigen::Matrix3d rota = Sophus::SO3d::exp(omega).matrix();

    // Matrix3d rota = Sophus :: SO3 <double> SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)).matrix();

    //	Vector2d tp1,residualV;
    //	Vector3d tp23d;
    //    for (int i = 0; i < int(lll.size()); i++) {
    //        tp1 = Vector2d(rr[i].x,rr[i].y);
    //        tp23d = Vector3d(lll[i].x,lll[i].y,lll[i].z);
    //
    //        tp23d = rota * tp23d + tran;
    //        Vector2d tp2(tp23d.x()/tp23d.z(),tp23d.y()/tp23d.z());
    //
    //        residualV = (tp2 - tp1);
    //        ROS_ERROR_STREAM(residualV.transpose());
    //    }
    ROS_DEBUG_STREAM("-----------pnp-----------");
    ROS_DEBUG_STREAM(lll.size());
    ROS_DEBUG_STREAM(inliersArr.rows);

    Rotation = rota.transpose();
    Translation = -rota.transpose() * tran;

    ROS_DEBUG_STREAM(Rotation);
    ROS_DEBUG_STREAM(Translation);
    return true;
}

bool MotionEstimator::solveRelativeRT_ICP(Eigen::aligned_vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &corres,
                                          Eigen::Matrix3d &R,
                                          Eigen::Vector3d &T)
{
    // refer to corres[i].first
    LOG(INFO) << "Solve relative pose using ICP";
    Eigen::Vector3d p1, p2;
    p1 = Eigen::Vec3d::Zero();
    p2 = Eigen::Vec3d::Zero();
    int N = corres.size();
    for (int i = 0; i < N; i++)
    {
        p1 += corres[i].first;
        p2 += corres[i].second;
    }
    p1 /= N;
    p2 /= N;

    Eigen::aligned_vector<Eigen::Vec3d> q1(N), q2(N);
    for (int i = 0; i < N; i++)
    {
        q1[i] = corres[i].first - p1;
        q2[i] = corres[i].second - p2;
    }
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vec3d(q1[i](0), q1[i](1), q1[i](2)) *
             Eigen::Vec3d(q2[i](0), q2[i](1), q2[i](2)).transpose();
    }

    // SVD
    LOG(INFO) << "ICP SVD";
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d tmp_R;
    Eigen::Vector3d tmp_T;
    tmp_R = U * V.transpose();
    tmp_T = p1 - tmp_R * p2;
    LOG(INFO) << "R:" << tmp_R;
    LOG(INFO) << "T:" << tmp_T;
    // Check uses reprojection error

    // Eigen::Vector3d residual = Eigen::Vec3d::Zero();
    double all_errors = 0;
    for (int i = 0; i < N; i++)
    {
        Eigen::Vector3d residual = corres[i].first - (tmp_R * corres[i].second + tmp_T);
        all_errors += residual.norm();
    }
    double ave_error = all_errors / N;
    if (ave_error > 1)
        return false;
    else
    {
        // TODO: convert to camera pose

        // R = tmp_R.transpose();
        // T = -R * tmp_T;
        R = tmp_R;
        T = tmp_T;
        LOG(INFO) << "Camera R \n"
                  << R;
        LOG(INFO) << "Camera T \n"
                  << T;
        return true;
    }
}

bool MotionEstimator::solveRelativeRT_PNP2(const Eigen::aligned_vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &corres,
                                           Eigen::Matrix3d &R,
                                           Eigen::Vector3d &T)
{   
    LOG(INFO)<<"Into relativeRT_PNP2";
    if (corres.size() > 15)
    {
        vector<cv::Point3f> pts3D;
        vector<cv::Point2f> pts2D;
        for (int i = 0; i < int(corres.size()); i++)
        {
            if (corres[i].first(2) > 0 && corres[i].second(2) > 0)
            {
                pts3D.push_back(cv::Point3f(corres[i].first(0), corres[i].first(1), corres[i].first(2)));
                pts2D.push_back(cv::Point2f(corres[i].second(0) / corres[i].second(2),
                                            corres[i].second(1) / corres[i].second(2)));
            }
        }
        cv::Mat r, rvec, t, D, tmp_r;
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        // cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        if (!cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 0))
        {
            LOG(INFO) << "Pnp failed!";
            return false;
        }

        cv::Rodrigues(rvec, r);
        Eigen::MatrixXd R_pnp;
        cv::cv2eigen(r, R_pnp);
        Eigen::MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);

        // cam_T_w ---> w_T_cam
        R = R_pnp.transpose();
        T = R * (-T_pnp);

        return true;
    }
    LOG(INFO)<<"Not enough features";
    return false;
}