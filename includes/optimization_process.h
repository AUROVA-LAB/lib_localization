#pragma once

#include "ceres_structs.h"

namespace geo_referencing {

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
	void addPriorConstraint (PriorConstraint constraint_prior){
		constraints_prior_.push_back(constraint_prior);
		if (constraints_prior_.size() > params_.window_size){
			constraints_prior_.erase(constraints_prior_.begin());
		}
	}
	void addAssoConstraint (AssoConstraint constraints_asso){
		constraints_asso_.push_back(constraints_asso);
		if (constraints_asso_.size() > params_.window_size){
			constraints_asso_.erase(constraints_asso_.begin());
		}
	}
	void addPose3dToTrajectoryEstimated (Pose3dWithCovariance pose3d_estimated){
		trajectory_estimated_.push_back(pose3d_estimated);
		if (trajectory_estimated_.size() > params_.window_size){
			trajectory_estimated_.erase(trajectory_estimated_.begin());
		}
	}
	void addTranslationTransform (float tf_trs){
		var_trs_.push_back(tf_trs);
		if (var_trs_.size() > params_.window_size / 5){
			var_trs_.erase(var_trs_.begin());
		}
	}
	void addRotationTransform (float tf_rot){
		var_rot_.push_back(tf_rot);
		if (var_rot_.size() > params_.window_size / 5){
			var_rot_.erase(var_rot_.begin());
		}
	}

	//// GET METHODS
	Trajectory getTrajectoryEstimated (void){
		return trajectory_estimated_;
	}
	PriorConstraintVector getPriorConstraints (void){
		return constraints_prior_;
	}
	float getTranslationVariance (void){
		float acum = 0.0;
		for (int i = 0; i < var_trs_.size(); i++) acum = acum + var_trs_.at(i);
		return acum / var_trs_.size();
	}
	float getRotationVariance (void){
		float acum = 0.0;
		for (int i = 0; i < var_rot_.size(); i++) acum = acum + var_rot_.at(i);
		return acum / var_rot_.size();
	}

	//// CLASS METHODS
	void generateOdomResiduals (ceres::LossFunction* loss_function,
			                    ceres::LocalParameterization* quaternion_local_parameterization,
								ceres::Problem* problem);
	void generatePriorResiduals (ceres::LossFunction* loss_function,
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

    Trajectory trajectory_estimated_;

	std::vector<float> var_trs_;
	std::vector<float> var_rot_;
};

}