#pragma once

#include <Eigen/Dense>
#include <vector>

namespace geo_referencing {

/**
 * @brief OdometryConstraint: The Constraint for odometry in the pose graph
 */
struct OdometryConstraint {
	size_t id_begin, id_end;

    // transformation from begin pose to end pose
	Eigen::Vector3d tf_p;
    Eigen::Quaterniond tf_q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Covariance and information matrix (inverse of covariance)
    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> information;
};

struct Pose3dWithCovariance {
	size_t id;

	Eigen::Vector3d p;
    Eigen::Quaterniond q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> information;
};

using PriorConstraint = Pose3dWithCovariance;
using PriorConstraintVector = std::vector<PriorConstraint>;
using OdometryConstraintsVector = std::vector<OdometryConstraint>;
using Trajectory = std::vector<Pose3dWithCovariance>;

struct ConfigParams {
	int window_size;
};

} // namespace geo_referencing