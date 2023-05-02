#include "../includes/optimization_process.h"

using namespace geo_referencing;

OptimizationProcess::OptimizationProcess(ConfigParams params)
{
    this->params_ = params;

    return;
}

void OptimizationProcess::generateOdomResiduals(ceres::LossFunction* loss_function,
		                                        ceres::LocalParameterization* quaternion_local_parameterization,
												ceres::Problem* problem)
{
    return;
}

void OptimizationProcess::initializeState(void)
{
    //// Generate transform
    Eigen::Matrix<double, 3, 1> p_ab_estimated(0.0, 0.0, 0.0);
    Eigen::AngleAxisd rot(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q_ab_estimated(rot);

    ////Propagate pose
    Pose3dWithCovariance propagated_pose;
    propagated_pose.id = 0;
    propagated_pose.p = p_ab_estimated;
    propagated_pose.q = q_ab_estimated;

    // Configure initial covariance
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    covariance(0, 0) = 0.1;
    covariance(1, 1) = 0.1;
    covariance(5, 5) = 0.05;

    //// Rotate initial covariance
    //Eigen::Matrix<double, 3, 3> r = trajectory_estimated_.at(index).q.toRotationMatrix();
    //Eigen::Matrix<double, 3, 3> b = covariance.block<3, 3>(0, 0);
    //covariance.block<3, 3>(0, 0) = r * b * r.transpose();
    propagated_pose.covariance = covariance;

    this->addPose3dToTrajectoryEstimated(propagated_pose);

    return;
}

void OptimizationProcess::propagateState (Eigen::Matrix<double, 3, 1> p_a, Eigen::Quaternion<double> q_a, 
                                          Eigen::Matrix<double, 3, 1> p_b, Eigen::Quaternion<double> q_b, int id)
{
    // Compute the relative transformation between the two frames.
    //Eigen::Quaternion<double> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<double> q_ab_estimated = q_b.inverse() * q_a;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<double, 3, 1> p_ab_estimated = p_b - p_a;
    ////
    
    ////Propagate pose
    Pose3dWithCovariance propagated_pose;
    propagated_pose.id = id;
    propagated_pose.p = trajectory_estimated_.at(trajectory_estimated_.size()-1).p + p_ab_estimated;
    propagated_pose.q = trajectory_estimated_.at(trajectory_estimated_.size()-1).q * q_ab_estimated;
    propagated_pose.covariance = trajectory_estimated_.at(trajectory_estimated_.size()-1).covariance;

    this->addPose3dToTrajectoryEstimated(propagated_pose);

    return;
}