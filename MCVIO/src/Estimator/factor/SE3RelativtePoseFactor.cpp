//
// Created by ubuntu on 2020/5/30.
//

#include "SE3RelativtePoseFactor.h"
#include "../../utility/sophus_utils.hpp"
#include "glog/logging.h"
bool SE3RelativtePoseFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Sophus::SE3d> T_w_from(parameters[0]);
    Eigen::Map<const Sophus::SE3d> T_w_end(parameters[1]);
    Eigen::Map<Sophus::Vector6d> res(residuals);

    // LOG(ERROR) << "Parameter0 type: " << Trait::getType(parameters[0] + 7);
    // LOG(INFO) << "Parameter0 id" << Trait::getId(parameters[0] + 7);
    // LOG(ERROR) << "Parameter1 type: " << Trait::getType(parameters[1] + 7);
    // LOG(INFO) << "Parameter1 id" << Trait::getId(parameters[1] + 7);

    Sophus::SE3d T_from_end_estimate = T_w_from.inverse() * T_w_end;

    // residual p
    res.segment<3>(0) = T_from_end_meas_.translation() - T_from_end_estimate.translation();
    // residual R
    res.segment<3>(3) = (T_from_end_meas_.so3() * T_from_end_estimate.so3().inverse()).log();

    Eigen::Vector3d phi = res.segment<3>(3);

    Eigen::Matrix3d Jrinv;
    Sophus::rightJacobianInvSO3(phi, Jrinv);

    res = sqrt_information_ * res;

    if (jacobians) {

        Eigen::Matrix3d Ri_inv = T_w_from.rotationMatrix().transpose();
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 8, Eigen::RowMajor>> jacobian_from(jacobians[0]);
            jacobian_from.setZero();

            jacobian_from.block<3, 3>(0, 0) = Ri_inv; // res_p_dpi
            jacobian_from.block<3, 3>(0, 3) = -Sophus::SO3d::hat(T_from_end_estimate.translation()) * Ri_inv;
            jacobian_from.block<3, 3>(3, 3) = Jrinv * Ri_inv; // res_R_dRi

            jacobian_from = sqrt_information_ * jacobian_from;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 6, 8, Eigen::RowMajor>> jacobian_end(jacobians[1]);
            jacobian_end.setZero();

            jacobian_end.block<3, 3>(0, 0) = -Ri_inv; //res_p_dpj
            jacobian_end.block<3, 3>(3, 3) = -Jrinv * Ri_inv; // //res_R_dRj


            jacobian_end = sqrt_information_ * jacobian_end;
        }
    }

    return true;
}
