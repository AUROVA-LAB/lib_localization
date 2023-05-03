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
	//// Generate residuals
	for (int j = 1; j < trajectory_estimated_.size(); j++){

		size_t index = j;

		ceres::CostFunction* cost_function_odom = OdometryErrorTerm::Create(constraints_odom_.at(index).tf_p,
																			constraints_odom_.at(index).tf_q,
																			constraints_odom_.at(index).information);
		problem->AddResidualBlock(cost_function_odom,
								  loss_function,
								  trajectory_estimated_.at(index-1).p.data(),
								  trajectory_estimated_.at(index-1).q.coeffs().data(),
								  trajectory_estimated_.at(index).p.data(),
								  trajectory_estimated_.at(index).q.coeffs().data());

        //std::cout << "A: " << trajectory_estimated_.at(index-1).id << " == " << constraints_odom_.at(index).id_begin << std::endl;
        //std::cout << "B: " << trajectory_estimated_.at(index).id << " == " << constraints_odom_.at(index).id_end << std::endl;

		problem->SetParameterization(trajectory_estimated_.at(index-1).q.coeffs().data(), quaternion_local_parameterization);
		problem->SetParameterization(trajectory_estimated_.at(index).q.coeffs().data(), quaternion_local_parameterization);
	}
	return;
}

void OptimizationProcess::generatePriorResiduals(ceres::LossFunction* loss_function,
		                                         ceres::LocalParameterization* quaternion_local_parameterization,
												 ceres::Problem* problem)
{
	//// Generate residuals
    for(int i = 0; i < constraints_prior_.size(); i++){
        for (int j = 0; j < trajectory_estimated_.size(); j++){
            if (constraints_prior_.at(i).id == trajectory_estimated_.at(j).id){
                ceres::CostFunction* cost_function_prior = PriorErrorTerm::Create(constraints_prior_.at(i).p,
                                                                                  constraints_prior_.at(i).information.block<3, 3>(0, 0));
                problem->AddResidualBlock(cost_function_prior,
                                          loss_function,
                                          trajectory_estimated_.at(j).p.data());
            }
        }
    }
	return;
}

void OptimizationProcess::solveOptimizationProblem(ceres::Problem* problem)
{
    //CHECK(problem != NULL);
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    std::cout << "Pre-solve" << std::endl;
    ceres::Solve(options, problem, &summary);
    std::cout << "Post-solve" << std::endl;
    //std::cout << summary.FullReport() << '\n';
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
    propagated_pose.id = -1;
    propagated_pose.p = p_ab_estimated;
    propagated_pose.q = q_ab_estimated;

    // Configure initial covariance
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
    propagated_pose.covariance = covariance;

    this->addPose3dToTrajectoryEstimated(propagated_pose);

    OdometryConstraint constraint_odom;
    constraint_odom.id_begin = -1;
    constraint_odom.id_end = 0;
    constraint_odom.tf_q = q_ab_estimated;
    constraint_odom.tf_p = p_ab_estimated;
    constraint_odom.covariance = covariance;
    constraint_odom.information = constraint_odom.covariance.inverse();

    this->addOdometryConstraint (constraint_odom);

    return;
}

void OptimizationProcess::propagateState (Eigen::Matrix<double, 3, 1> p_a, Eigen::Quaternion<double> q_a, 
                                          Eigen::Matrix<double, 3, 1> p_b, Eigen::Quaternion<double> q_b, int id)
{
    // Compute the relative transformation between the two frames.
    //Eigen::Quaternion<double> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<double> q_ab_estimated = q_a.conjugate() * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Quaternion<double> a_frames_diff = q_a * trajectory_estimated_.at(trajectory_estimated_.size()-1).q.conjugate();
    Eigen::Matrix<double, 3, 1> p_ab_estimated = a_frames_diff.conjugate() * (p_b - p_a);
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