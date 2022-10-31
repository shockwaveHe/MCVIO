//
// Created by minxuan, xubo on 2020/5/30.
//

#ifndef TYPES_H
#define TYPES_H

#include <istream>
#include <string>
#include <array>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


struct Pose3d
{
    static constexpr size_t keyBits = sizeof(int64_t) * 8;
    static constexpr size_t chrBits = sizeof(unsigned char) * 8;
    static constexpr size_t indexBits = keyBits - chrBits;
    static constexpr int64_t charMask = int64_t(255) << indexBits;
    static constexpr int64_t indexMask = ~charMask;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose3d()
    {
        std::memset(&data[0], 0, 8 * sizeof(double));
        data[3] = 1.0;
    }

    static std::string name()
    {
        return "VERTEX_SE3:QUAT";
    }

    Eigen::Quaterniond rot() const
    {
        return Eigen::Map<const Eigen::Quaterniond>(data.data());
    }

    Eigen::Vector3d pos() const
    {
        return Eigen::Map<const Eigen::Vector3d>(data.data() + 4);
    };

    void setRot(const Eigen::Quaterniond &quat)
    {

        Eigen::Map<Eigen::Quaterniond>(data.data()) = quat;
    }

    void setRot(const Eigen::Matrix3d &R)
    {
        Eigen::Map<Eigen::Quaterniond>(data.data()) = Eigen::Quaterniond(R);
    }

    void setPos(const Eigen::Vector3d &pos)
    {
        Eigen::Map<Eigen::Vector3d>(data.data() + 4) = pos;
    }

    void setId(int64_t id)
    {
        *(int64_t *) (data.data() + 7) = (int64_t('p') << indexBits) | id;
    }

    char getType()
    {
        return char((*(int64_t *) (data.data() + 7) & charMask) >> indexBits);
    }

    int64_t getId()
    {
        return *(int64_t *) (data.data() + 7) & indexMask;
    }

    friend inline std::istream &
    operator>>(std::istream &input, Pose3d &pose)
    {
        input >> pose.data[4] >> pose.data[5] >> pose.data[6]
              >> pose.data[0] >> pose.data[1] >> pose.data[2] >> pose.data[3];

        // Normalize the quaternion to account for precision loss due to
        // serialization.
        Eigen::Map<Eigen::Quaterniond>(pose.data.data()).normalized();

        return input;
    }
public:
    // ceres的ParameterBlock接口
    // qx, qy, qz, qw, x, y, z, id
    std::array<double, 8> data;
}; // class Pose3d

typedef std::map<int, Pose3d, std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, Pose3d>>>

    MapOfPoses;

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.

struct Constraint3d
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // The name of the data type in the g2o file format.
    static std::string name()
    {
        return "EDGE_SE3:QUAT";
    }

    int host_id;
    int target_id;

    // The transformation that represents the pose of the end frame E w.r.t. the
    // begin frame B. In other words, it transforms a vector in the E frame to
    // the B frame.

    Pose3d T_h_t;

    // The inverse of the covariance matrix for the measurement. The order of the
    // entries are x, y, z, delta orientation.

    Eigen::Matrix<double, 6, 6> information;
}; // struct Constraint3d

inline std::istream &
operator>>(std::istream &input, Constraint3d &constraint)
{
    Pose3d &T_h_t = constraint.T_h_t;
    input >> constraint.host_id >> constraint.target_id >> T_h_t;

    for (int i = 0; i < 6 && input.good(); ++i) {
        for (int j = i; j < 6 && input.good(); ++j) {
            input >> constraint.information(i, j);
            if (i != j)
                constraint.information(j, i) = constraint.information(i, j);
        }
    }

    return input;
}

typedef std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>>
    VectorOfConstraints;

#endif //TYPES_H
