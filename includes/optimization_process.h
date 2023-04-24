#include "ceres_structs.h"

namespace geo_referencing {

class OptimizationProcess {
public:
	OptimizationProcess(ConfigParams params);
	~OptimizationProcess() { }

	void addOdometryConstraint (OdometryConstraint constraint_odom){
		constraints_odom_.push_back(constraint_odom);
		if (constraints_odom_.size() > params_.window_size){
			constraints_odom_.erase(constraints_odom_.begin());
		}
	}

	void generateOdomResiduals (ceres::LossFunction* loss_function,
			                    ceres::LocalParameterization* quaternion_local_parameterization,
								ceres::Problem* problem);
private:
    ConfigParams params_;

	OdometryConstraintsVector constraints_odom_;
    Trajectory trajectory_estimated_;
};

}