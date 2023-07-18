#pragma once

#include "ceres_structs.h"

namespace optimization_process {

class OptimizationProcess {
public:
	OptimizationProcess(ConfigParams params);
	~OptimizationProcess() { }

	//// SET METHODS
	void addOdometryConstraint (OdometryConstraint constraint_odom){
		constraints_odom_.push_back(constraint_odom);
		if (constraints_odom_.size() > params_.window_size){
			constraints_odom_.erase(constraints_odom_.begin());
		}
	}
	void addOdometryConstraintGT (OdometryConstraint constraint_odom){
		constraints_odom_GT_.push_back(constraint_odom);
	}
	void addPriorConstraint (PriorConstraint constraint_prior){
		constraints_prior_.push_back(constraint_prior);
		if (constraints_prior_.size() > params_.window_size){
			constraints_prior_.erase(constraints_prior_.begin());
		}
	}
	void addPriorConstraintGT (PriorConstraint constraint_prior){
		constraints_prior_GT_.push_back(constraint_prior);
	}
	void addAssoConstraint (AssoConstraint constraints_asso){
		constraints_asso_.push_back(constraints_asso);
		if (constraints_asso_.size() > params_.window_size){
			constraints_asso_.erase(constraints_asso_.begin());
		}
	}
	void addAssoPointConstraintsSingleShot (AssoPointsConstraintsSingleShot constraints_asso_pt_ss){
		constraints_asso_pt_vc_.push_back(constraints_asso_pt_ss);
		if (constraints_asso_pt_vc_.size() > params_.window_size){
			constraints_asso_pt_vc_.erase(constraints_asso_pt_vc_.begin());
		}
	}
	void addPose3dToTrajectoryEstimated (Pose3dWithCovariance pose3d_estimated){
		trajectory_estimated_.push_back(pose3d_estimated);
		if (trajectory_estimated_.size() > params_.window_size){
			trajectory_estimated_.erase(trajectory_estimated_.begin());
		}
	}
	void addPose3dToTrajectoryEstimatedGT (Pose3dWithCovariance pose3d_estimated){
		trajectory_estimated_GT_.push_back(pose3d_estimated);
	}

	//// GET METHODS
	Trajectory getTrajectoryEstimated (void){
		return trajectory_estimated_;
	}
	Trajectory getTrajectoryEstimatedGT (void){
		return trajectory_estimated_GT_;
	}
	PriorConstraintVector getPriorConstraints (void){
		return constraints_prior_;
	}
	Eigen::Vector3d getPriorError (void){
		return prior_error_;
	}

	//// CLASS METHODS
	void generateOdomResiduals (ceres::LossFunction* loss_function,
			                    ceres::LocalParameterization* quaternion_local_parameterization,
								ceres::Problem* problem);
	void generatePriorResiduals (ceres::LossFunction* loss_function,
			                     ceres::LocalParameterization* quaternion_local_parameterization,
								 ceres::Problem* problem);
	void generateOdomResidualsGT (ceres::LossFunction* loss_function,
			                      ceres::LocalParameterization* quaternion_local_parameterization,
								  ceres::Problem* problem);
	void generatePriorResidualsGT (ceres::LossFunction* loss_function,
			                       ceres::LocalParameterization* quaternion_local_parameterization,
								   ceres::Problem* problem);
	void generatePriorErrorResiduals (ceres::LossFunction* loss_function,
			                          ceres::LocalParameterization* quaternion_local_parameterization,
								      ceres::Problem* problem);
	void generateAssoPointResiduals (ceres::LossFunction* loss_function,
			                         ceres::LocalParameterization* quaternion_local_parameterization,
								     ceres::Problem* problem);
	void generateAssoResiduals (ceres::LossFunction* loss_function,
			                    ceres::LocalParameterization* quaternion_local_parameterization,
								ceres::Problem* problem);
	void initializeState (void);
	void propagateState (Eigen::Matrix<double, 3, 1> p_a, Eigen::Quaternion<double> q_a, 
                         Eigen::Matrix<double, 3, 1> p_b, Eigen::Quaternion<double> q_b, int id);
	void solveOptimizationProblem (ceres::Problem* problem);

private:
    ConfigParams params_;

	OdometryConstraintsVector constraints_odom_;
	PriorConstraintVector constraints_prior_;
	AssoConstraintVector constraints_asso_;
	AssoPointsConstraintsVector constraints_asso_pt_vc_;

    Trajectory trajectory_estimated_;
	Eigen::Vector3d prior_error_;

	// Vaiable fot Ground Truth
	Trajectory trajectory_estimated_GT_;
	OdometryConstraintsVector constraints_odom_GT_;
	PriorConstraintVector constraints_prior_GT_;
};

}