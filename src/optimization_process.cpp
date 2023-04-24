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