#include "ceres_structs.h"

namespace geo_referencing {

class OptimizationProcess {
public:
	OptimizationProcess(ConfigParams params);
	~OptimizationProcess() { }

	//// SET FUNCTIONS
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
	void addPose3dToTrajectoryEstimated (Pose3dWithCovariance pose3d_estimated){
		trajectory_estimated_.push_back(pose3d_estimated);
		if (trajectory_estimated_.size() > params_.window_size){
			trajectory_estimated_.erase(trajectory_estimated_.begin());
		}
	}

	//// GET FUNCTIONS
	Trajectory getTrajectoryEstimated (void){
		return trajectory_estimated_;
	}

	//// CLASS FUNCTIONS
	void generateOdomResiduals (ceres::LossFunction* loss_function,
			                    ceres::LocalParameterization* quaternion_local_parameterization,
								ceres::Problem* problem);
	void generatePriorResiduals (ceres::LossFunction* loss_function,
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
    Trajectory trajectory_estimated_;
};

}