#ifndef UR5_H
#define UR5_H

#include <vector>
#include <eigen3/Eigen/Geometry>


namespace UR3 {
    using Configuration = std::array<double,6>;

    bool solveIK(const Eigen::AffineCompact3d &transform, std::vector<Configuration> &theta,
                 const Configuration &theta_ref, const Configuration &theta_min,
                 const Configuration &theta_max);
}


#endif // UR5_H
