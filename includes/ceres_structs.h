#pragma once

#include "types_optimization.h"
#include "ceres/ceres.h"


namespace optimization_process {

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
        //Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a.conjugate() * q_b;

        // Represent the displacement between the two frames in the A frame.
        //Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a.conjugate() * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q = tf_q_.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = (p_ab_estimated - tf_p_.template cast<T>()) * 100.0;
        residuals.template block<3, 1>(3, 0) = delta_q.vec() * 100.0;

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

/**
 * @brief: prior (gnss) cost function
 */
struct PriorErrorTerm {
	PriorErrorTerm(const Eigen::Vector3d& p_prior, const Eigen::Matrix<double, 3, 3>& information)
            : p_prior_(p_prior), information_(information) {
    }

    template <typename T>
    bool operator()(const T* p_ptr, T* residuals_ptr) const {
        Eigen::Matrix<T, 3, 1> p(p_ptr);

        // compute the residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        Eigen::Matrix<T, 3, 1> delta = p - p_prior_.template cast<T>();
        residuals.template block<3, 1>(0, 0) = delta;

        // Scale the residuals by the measurement uncertainty.
		residuals.applyOnTheLeft(information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& p_prior,
                                       const Eigen::Matrix<double, 3, 3>& information) {
        return new ceres::AutoDiffCostFunction<PriorErrorTerm, 3, 3>(new PriorErrorTerm(p_prior, information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // The prior point (gnss).
    const Eigen::Vector3d p_prior_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 3, 3> information_;
};

/**
 * @brief: prior (gnss) misalignment cost function
 */
struct PriorMisErrorTerm {
	PriorMisErrorTerm(const Eigen::Vector3d& p_prior, const Eigen::Vector3d& p_state, const Eigen::Matrix<double, 3, 3>& information)
            : p_prior_(p_prior), p_state_(p_state), information_(information) {
    }

    template <typename T>
    bool operator()(const T* e_ptr, T* residuals_ptr) const {
        Eigen::Matrix<T, 3, 1> e(e_ptr);

        // compute the residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        Eigen::Matrix<T, 3, 1> delta = p_state_.template cast<T>() - (p_prior_.template cast<T>() - e);
        residuals.template block<3, 1>(0, 0) = delta;

        // Scale the residuals by the measurement uncertainty.
		residuals.applyOnTheLeft(information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& p_prior,
                                       const Eigen::Vector3d& p_state,
                                       const Eigen::Matrix<double, 3, 3>& information) {
        return new ceres::AutoDiffCostFunction<PriorMisErrorTerm, 3, 3>(new PriorMisErrorTerm(p_prior, p_state, information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // The prior point (gnss).
    const Eigen::Vector3d p_prior_;
    // The state point (localization).
    const Eigen::Vector3d p_state_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 3, 3> information_;
};

/**
 * @brief: data association points cost function
 */
struct AssoPointsErrorTerm {
	AssoPointsErrorTerm(const Eigen::Vector3d& det, const Eigen::Vector3d& lm, const double& asso_weight, const Eigen::Matrix<double, 3, 3>& information)
            : det_(det), lm_(lm), asso_weight_(asso_weight), information_(information) {
    }

    template <typename T>
    bool operator()(const T* p_ptr, const T* q_ptr, T* residuals_ptr) const {
        Eigen::Matrix<T, 3, 1> p(p_ptr);
        Eigen::Quaternion<T> q(q_ptr);

        // transform pq to tf matrix
        Eigen::Transform<T, 3, Eigen::Isometry, Eigen::DontAlign> tf;
        Eigen::Matrix<T, 3, 3> r = q.toRotationMatrix();
        Eigen::Matrix<T, 3, 1> t = p;
        tf.linear() = r;
        tf.translation() = t;

        // compute the residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        Eigen::Matrix<T, 3, 1> det_tf = tf * det_.template cast<T>();
        Eigen::Matrix<T, 3, 1> delta = det_tf - lm_.template cast<T>();
        residuals.template block<3, 1>(0, 0) = delta * asso_weight_;

        // Scale the residuals by the measurement uncertainty.
		residuals.applyOnTheLeft(information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& det,
                                       const Eigen::Vector3d& lm,
                                       const double& asso_weight,
                                       const Eigen::Matrix<double, 3, 3>& information) {
        return new ceres::AutoDiffCostFunction<AssoPointsErrorTerm, 3, 3, 4>(new AssoPointsErrorTerm(det, lm, asso_weight, information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // the matched landmark and detection.
    const Eigen::Vector3d det_;
    const Eigen::Vector3d lm_;
    const double asso_weight_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 3, 3> information_;
};

/**
 * @brief: data association tf cost function
 */
struct AssoErrorTerm {
	AssoErrorTerm(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, const Eigen::Matrix<double, 6, 6>& information)
            : p_(p), q_(q), information_(information) {
    }

    template <typename T>
    bool operator()(const T* p_ptr, const T* q_ptr, T* residuals_ptr) const {
        Eigen::Matrix<T, 3, 1> p_est(p_ptr);
        Eigen::Quaternion<T> q_est(q_ptr);


        // compute the residual
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        Eigen::Matrix<T, 3, 1> delta_p = p_est - p_.template cast<T>();
        Eigen::Quaternion<T> delta_q = q_.template cast<T>() * q_est.conjugate();
        
        residuals.template block<3, 1>(0, 0) = delta_p;
        residuals.template block<3, 1>(3, 0) = delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
		residuals.applyOnTheLeft(information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& p,
                                       const Eigen::Quaterniond& q,
                                       const Eigen::Matrix<double, 6, 6>& information) {
        return new ceres::AutoDiffCostFunction<AssoErrorTerm, 6, 3, 4>(new AssoErrorTerm(p, q, information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // The prior point (gnss).
    const Eigen::Vector3d p_;
    const Eigen::Quaterniond q_;

    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> information_;
};

} // namespace optimization_process