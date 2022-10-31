//
// Created by ubuntu on 2020/5/30.
//

#include "FrameParameterization.h"
// Parameterization

FrameParameterization::FrameParameterization()
{

}

FrameParameterization::~FrameParameterization()
{

}

bool FrameParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    //map buffers
    Eigen::Map<Eigen::Quaterniond const> q(x);
    Eigen::Map<Eigen::Vector3d const> t(x + 4);

    Eigen::Map<Eigen::Quaterniond> q_pluse_delta(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> t_plus_delta(x_plus_delta + 4);

    Eigen::Vector3d delta_t = Eigen::Map<Eigen::Vector3d const>(delta);
    Eigen::Quaterniond delta_q = Sophus::SO3d::exp(Eigen::Map<Eigen::Vector3d const>(delta + 3)).unit_quaternion();

    //    LOG(INFO) << "increment d: " << d.transpose();

    q_pluse_delta = (delta_q* q).normalized();
    t_plus_delta = t + delta_t;
    x_plus_delta[7] = x[7];

    return true;
}

bool FrameParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    // trick to work directly in the tangent space
    // compute jacobians relativate to the tangent space
    // and let this jacobian be the identity
    // J = [I(6x6); 0]

    Eigen::Map<Eigen::Matrix<double, 8, 6, Eigen::RowMajor>> J(jacobian);
    J.setZero();
    J.block<6, 6>(0, 0).setIdentity();
    return true;
}

int FrameParameterization::GlobalSize() const
{
    return (Sophus::SE3d::num_parameters + 1);
}

int FrameParameterization::LocalSize() const
{
    return (Sophus::SE3d::DoF);
}