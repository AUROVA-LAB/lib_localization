#include "../includes/association_problem.h"

namespace static_data_association{

AssociationProblem::AssociationProblem(void) {
    precalculated_ = false;
    return;
}

void AssociationProblem::clearDetections(void) {
    detections_.clear();
    return;
}

void AssociationProblem::clearLandmarks(void) {
    landmarks_.clear();
    return;
}

bool AssociationProblem::areDistanceCompatible(const size_t i1, const size_t j1, const size_t i2, const size_t j2, const double distance) const {
    if(!precalculated_) {
        precalc();
    }
    return std::abs(detectionDistanceCache_(i1, i2) - landmarkDistanceCache_(j1, j2)) < distance;
}

void AssociationProblem::precalc() const {
    detectionDistanceCache_ = Eigen::MatrixXd(detections_.size(), detections_.size());
    for(size_t i1 = 0; i1 < detections_.size(); ++i1) {
        for(size_t i2 = 0; i2 < detections_.size(); ++i2) {
            detectionDistanceCache_(i1,i2) = detections_[i1]->distanceBetweenDetections(*(detections_[i2]));
        }
    }
    landmarkDistanceCache_ = Eigen::MatrixXd(landmarks_.size(), landmarks_.size());
    for(size_t i1 = 0; i1 < landmarks_.size(); ++i1) {
        for(size_t i2 = 0; i2 < landmarks_.size(); ++i2) {
            landmarkDistanceCache_(i1,i2) = landmarks_[i1]->distanceBetweenDetections(*(landmarks_[i2]));
        }
    }
    precalculated_ = true;
}

Hypothesis AssociationProblem::dcsacMethod(SacBasedCfg sac_cfg, Eigen::Isometry3d tf) {

    //AssociationProblem sac_asso(sac_cfg, *this);

    Hypothesis best(*this);

    for (int i = 0; i < landmarks_.size(); i+=4){
        for (int j = 0; j < detections_.size(); j++){
            float x_lm = landmarks_.at(i)->position().x();
            float y_lm = landmarks_.at(i)->position().y();
            float x_dt = detections_.at(j)->position().x();
            float y_dt = detections_.at(j)->position().y();
            if (sqrt(pow(x_lm-x_dt, 2) + pow(y_lm-y_dt, 2)) < 5.0){ //// TODO: Get from param
                for (int u = 0; u < landmarks_.size(); u++){
                    for (int v = 0; v < detections_.size(); v++){
                        float distance = landmarks_.at(u)->position().y() - detections_.at(v)->position().y();
                    }
                }
            }
        }
    }

    /*sac_asso.dcsacImplementation(tf);
    
	for (size_t i = 0; i < sac_asso.getAssociations().size(); i++)
	{
		size_t d_id = sac_asso.getAssociations()[i].first;
		size_t l_id = sac_asso.getAssociations()[i].second;
		best = Hypothesis(best, d_id, l_id);
	}*/

    return best;
}

}