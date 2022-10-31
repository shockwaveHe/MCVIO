//
// Created by ubuntu on 2020/5/30.
//

#ifndef SE3RELATIVTEPOSEFACTOR_H
#define SE3RELATIVTEPOSEFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include <sophus/se3.hpp>
#include <utility>
#include "../../utility/Twist.h"

struct Trait
{
    static constexpr size_t keyBits = sizeof(int64_t) * 8;
    static constexpr size_t chrBits = sizeof(unsigned char) * 8;
    static constexpr size_t indexBits = keyBits - chrBits;
    static constexpr int64_t charMask = int64_t(255) << indexBits;
    static constexpr int64_t indexMask = ~charMask;

    static char
    getType(const double *const data)
    {
        return char((*(int64_t *) (data) & charMask) >> indexBits);
    }

    static
    int64_t getId(const double *const data)
    {
        return *(int64_t *) (data) & indexMask;
    }
};

class SE3RelativtePoseFactor: public ceres::SizedCostFunction<6, 8, 8>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SE3RelativtePoseFactor(const Sophus::SE3d &T_from_end_meas, const Sophus::Matrix6d &sqrt_information)
        : T_from_end_meas_(T_from_end_meas),
          sqrt_information_(sqrt_information)
    {

    }

    virtual bool
    Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

private:
    const Sophus::SE3d T_from_end_meas_;
    const Sophus::Matrix6d sqrt_information_;
};


#endif //SE3RELATIVTEPOSEFACTOR_H
