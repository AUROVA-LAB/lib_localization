#pragma once

#include <Eigen/Dense>
#include <vector>

namespace optimization_process {

/**
 * @brief OdometryConstraint: The Constraint for odometry in the pose graph
 */
struct OdometryConstraint {
	size_t id_begin, id_end;

    // transformation from begin pose to end pose
	Eigen::Vector3d tf_p;
    Eigen::Quaterniond tf_q;
    double odom_weight;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Covariance and information matrix (inverse of covariance)
    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> information;
};

/**
 * @brief AssoPointsConstraint: The Constraint for points associations in the pose graph
 */
struct AssoPointsConstraint {
    size_t id;

    // Associate data
    Eigen::Vector3d detection;
    Eigen::Vector3d landmark;
    double asso_weight;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Covariance and information matrix (inverse of covariance)
    Eigen::Matrix<double, 3, 3> covariance;
    Eigen::Matrix<double, 3, 3> information;
};

struct Pose3dWithCovariance {
	size_t id;

	Eigen::Vector3d p;
    Eigen::Quaterniond q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> information;
};

struct PriorConstraint {
	size_t id;

	Eigen::Vector3d p;
    Eigen::Vector3d p_raw;
    Eigen::Quaterniond q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, 3, 3> covariance;
    Eigen::Matrix<double, 3, 3> information;
};

using PriorConstraintVector = std::vector<PriorConstraint>;

using AssoConstraint = Pose3dWithCovariance;
using AssoConstraintVector = std::vector<AssoConstraint>;

using AssoPointsConstraintsSingleShot = std::vector<AssoPointsConstraint>;
using AssoPointsConstraintsVector = std::vector<AssoPointsConstraintsSingleShot>;

using OdometryConstraintsVector = std::vector<OdometryConstraint>;
using Trajectory = std::vector<Pose3dWithCovariance>;

struct ConfigParams {
	int window_size;
    int max_num_iterations_op;
};

} // namespace optimization_process