#include "types_opt.h"
#include "ceres/ceres.h"


namespace geo_referencing {

/**
 * @brief: Odometry cost function
 */
struct OdometryErrorTerm {
	OdometryErrorTerm(const Eigen::Vector3d tf_p, const Eigen::Quaterniond tf_q, const Eigen::Matrix<double, 6, 6>& information)
            : tf_p_(tf_p), tf_q_(tf_q), information_(information){
    }

    template <typename T>
    bool operator()(const T* p_a_ptr, const T* q_a_ptr, const T* p_b_ptr, const T* q_b_ptr, T* residuals_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q = tf_q_.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = (p_ab_estimated - tf_p_.template cast<T>());
        residuals.template block<3, 1>(3, 0) = delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
		residuals.applyOnTheLeft(information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d tf_p, const Eigen::Quaterniond tf_q,
    		                           const Eigen::Matrix<double, 6, 6>& information) {
        return new ceres::AutoDiffCostFunction<OdometryErrorTerm, 6, 3, 4, 3, 4>(new OdometryErrorTerm(tf_p, tf_q, information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // The measurement for the position of B relative to A in the A frame.
	const Eigen::Vector3d tf_p_;
    const Eigen::Quaterniond tf_q_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> information_;
};

} // namespace geo_referencing