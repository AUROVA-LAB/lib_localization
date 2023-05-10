#pragma once

#include <random>

#include "igl/procrustes.h"
#include "association_problem.h"

//using namespace static_data_association;

namespace static_data_association{

template<int Dim>
class SacBasedAssociation {
public:
	SacBasedAssociation(SacBasedCfg sac_cfg, AssociationProblem<Dim>& ap) : ap_{ap} {
        // Check dimensionality
        static_assert((Dim == 2 || Dim == 3), "AssociationProblem is only implemented for dimensions 2 and 3.");
        sac_cfg_.error_threshold = sac_cfg.error_threshold;
        sac_cfg_.threshold_asso = sac_cfg.threshold_asso;
        sac_cfg_.compatibility_distance = sac_cfg.compatibility_distance;
        sac_cfg_.x_var = sac_cfg.x_var;
        sac_cfg_.y_var = sac_cfg.y_var;
        sac_cfg_.w_var = sac_cfg.w_var;
    }
    ~SacBasedAssociation() { }

    void addAssociation(std::pair<size_t, size_t> association) {
    	associations_.push_back(association);
    }
    std::vector<PairIds> getAssociations() {
    	return associations_;
    }

    void distanceCompatiblePairsSelection (void);
    void randomSamplePairsSelection (void);
    void calculateTfBetweenPairs (size_t idv_d, size_t idv_l, Eigen::Isometry3d& tf);
    float huberLoss (float error, float sigma);
    float distanceForLineSegments (std::vector<Eigen::Vector3d> detections_tf, int i, int j);
    float distanceForPoints (std::vector<Eigen::Vector3d> detections_tf, int i, int j);
    float computeTotalError (Eigen::Isometry3d tf, bool save_asso);
    void dcsacImplementation (Eigen::Isometry3d& tf_min);
    void ransacImplementation (Eigen::Isometry3d& tf_min);
    void nnImplementation (void);

private:
    std::vector<PairIds> associations_;
    std::vector<AssoCandidates> associations_candidates_;

    std::reference_wrapper<AssociationProblem<Dim> > ap_;
    SacBasedCfg sac_cfg_;
};

template<int Dim>
void SacBasedAssociation<Dim>::distanceCompatiblePairsSelection (void)
{

	/////////////////////////////////////////////////////////////////////////////////////////
	//// DETECTION PAIR SELECTION
	int N = ap_.get().getDetections().size();
	std::vector<int> i_first;
	std::vector<int> i_second;
	for (int i = 0; i < N; i++){
		i_first.push_back(i);
		i_second.push_back(i);
	}

	std::default_random_engine generator;
	std::uniform_real_distribution<double> random_var(0.0, 1.0);

	while (i_first.size() > 0){
		int index_i = std::round((i_first.size()-1) * random_var(generator));
		int index_j = std::round((i_second.size()-1) * random_var(generator));
		int i = i_first[index_i];
		int j = i_second[index_j];

		if (ap_.get().getDetections()[i]->individuallyCompatible(*(ap_.get().getDetections()[j]))){ // only individually comp. pairs
			i_first.erase(i_first.begin() + index_i);
			i_second.erase(i_second.begin() + index_j);

			AssoCandidates associations_candidates;

			associations_candidates.detect_pair_ids.first = i;
			associations_candidates.detect_pair_ids.second = j;

			associations_candidates_.push_back(associations_candidates);
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	//// LANDMARKS PAIR SELECTION DISTANCE COMPATIBLE
    for (int k = 0; k < associations_candidates_.size(); k++)
    {
    	PairIds detect_pair_ids = associations_candidates_[k].detect_pair_ids;
        for (int i = 0; i < ap_.get().getLandmarks().size(); i++)
        {
        	if (ap_.get().areIndividuallyCompatible(detect_pair_ids.first, i)) // only compatible with detect.
        	{
				for (int j = 0; j < ap_.get().getLandmarks().size(); j++)
				{
					if (ap_.get().getLandmarks()[i]->individuallyCompatible(*(ap_.get().getLandmarks()[j]))){ // only individually comp. pairs

						size_t i1 = detect_pair_ids.first;
						size_t j1 = i;
						size_t i2 = detect_pair_ids.second;
						size_t j2 = j;

						if (ap_.get().areDistanceCompatible(i1, j1, i2, j2, sac_cfg_.compatibility_distance)){
							PairIds landmark_pair_ids;
							landmark_pair_ids.first = i;
							landmark_pair_ids.second = j;
							associations_candidates_[k].landmarks_pairs_ids.push_back(landmark_pair_ids);
						}
					}
				}
        	}
        }
    }
	return;
}

template<int Dim>
void SacBasedAssociation<Dim>::randomSamplePairsSelection (void)
{
	/////////////////////////////////////////////////////////////////////////////////////////
	//// DETECTION RANDOM PAIR SELECTION
	int N = ap_.get().getDetections().size();
	std::vector<int> i_first;
	std::vector<int> i_second;
	for (int i = 0; i < N; i++){
		i_first.push_back(i);
		i_second.push_back(i);
	}

	std::default_random_engine generator;
	std::uniform_real_distribution<double> random_var(0.0, 1.0);

	while (i_first.size() > 0){
		int index_i = std::round((i_first.size()-1) * random_var(generator));
		int index_j = std::round((i_second.size()-1) * random_var(generator));
		int i = i_first[index_i];
		int j = i_second[index_j];

		if (ap_.get().getDetections()[i]->individuallyCompatible(*(ap_.get().getDetections()[j]))){ // only individually comp. pairs
			i_first.erase(i_first.begin() + index_i);
			i_second.erase(i_second.begin() + index_j);

			AssoCandidates associations_candidates;

			associations_candidates.detect_pair_ids.first = i;
			associations_candidates.detect_pair_ids.second = j;

			associations_candidates_.push_back(associations_candidates);
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	//// LANDMARKS RANDOM PAIR SELECTION
	float distance_allowed = sqrt(pow(sac_cfg_.x_var, 2) + pow(sac_cfg_.y_var, 2));
    for (int k = 0; k < associations_candidates_.size(); k++)
    {
    	PairIds detect_pair_ids = associations_candidates_[k].detect_pair_ids;

    	int M = ap_.get().getLandmarks().size();
    	i_first.clear();
    	for (int w = 0; w < M; w++) i_first.push_back(w);

    	while (i_first.size() > 0){

    		int index_i = std::round((i_first.size()-1) * random_var(generator));
    		int i = i_first[index_i];
    		i_first.erase(i_first.begin() + index_i);

    		float distance = ap_.get().getLandmarks()[i]->distanceBetweenDetections(*(ap_.get().getDetections()[detect_pair_ids.first]));

        	if (ap_.get().areIndividuallyCompatible(detect_pair_ids.first, i) &&  // only compatible with detect.
        		distance < distance_allowed){ // candidates selected depending on covariance matrix in localization

        		i_second.clear();
        		for (int w = 0; w < M; w++) i_second.push_back(w);
        		while (i_second.size() > 0){

        			int index_j = std::round((i_second.size()-1) * random_var(generator));
        			int j = i_second[index_j];
        			i_second.erase(i_second.begin() + index_j);

        			if ((j != i) && (ap_.get().getLandmarks()[i]->individuallyCompatible(*(ap_.get().getLandmarks()[j])))){ // only individually comp. pairs
        				PairIds landmark_pair_ids;
        				landmark_pair_ids.first = i;
        				landmark_pair_ids.second = j;
        				associations_candidates_[k].landmarks_pairs_ids.push_back(landmark_pair_ids);
        			}
        		}
        	}
    	}
    }

	return;
}

template<int Dim>
void SacBasedAssociation<Dim>::calculateTfBetweenPairs (size_t idv_d, size_t idv_l, Eigen::Isometry3d& tf)
{

    Eigen::MatrixX2d detections(2,2);
    Eigen::MatrixX2d landmarks(2,2);

    // Tf in 2d projection
    int idv_d_f = associations_candidates_[idv_d].detect_pair_ids.first;
    int idv_d_s = associations_candidates_[idv_d].detect_pair_ids.second;
    int idv_l_f = associations_candidates_[idv_d].landmarks_pairs_ids[idv_l].first;
    int idv_l_s = associations_candidates_[idv_d].landmarks_pairs_ids[idv_l].second;

    detections(0, 0) = ap_.get().getDetections()[idv_d_f]->position().x();
    detections(0, 1) = ap_.get().getDetections()[idv_d_f]->position().y();
    detections(1, 0) = ap_.get().getDetections()[idv_d_s]->position().x();
    detections(1, 1) = ap_.get().getDetections()[idv_d_s]->position().y();

    landmarks(0, 0) = ap_.get().getLandmarks()[idv_l_f]->position().x();
    landmarks(0, 1) = ap_.get().getLandmarks()[idv_l_f]->position().y();
    landmarks(1, 0) = ap_.get().getLandmarks()[idv_l_s]->position().x();
    landmarks(1, 1) = ap_.get().getLandmarks()[idv_l_s]->position().y();

    Eigen::Rotation2Dd rotation;
    Eigen::Vector2d translation;
    igl::procrustes(landmarks.block(0,0,2,2), detections.block(0,0,2,2), rotation, translation);

    Eigen::Isometry2d tf2d;
    tf2d = Eigen::Isometry2d::Identity();
    tf2d.rotate(rotation);
    tf2d.translate(-translation.head<2>());

    // Parse to 3d Tf
    Eigen::Vector3d translation3d = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation3d = Eigen::Matrix3d::Identity();
    translation3d.block<2, 1>(0, 0) =  tf2d.translation();
    rotation3d.block<2, 2>(0, 0) = tf2d.linear();

    tf= Eigen::Isometry3d::Identity();
    tf.rotate(rotation3d);
    tf.translate(-translation3d.head<3>());

	return;
}

template<int Dim>
float SacBasedAssociation<Dim>::distanceForPoints (std::vector<Eigen::Vector3d> detections_tf, int i, int j)
{
	DalmrPolyDetection detection_tf_obj(detections_tf.at(0), ap_.get().getDetections()[i]->identifier());

	float distance = ap_.get().getLandmarks()[j]->distanceBetweenDetections(detection_tf_obj);
	return distance;
}

template<int Dim>
float SacBasedAssociation<Dim>::distanceForLineSegments (std::vector<Eigen::Vector3d> detections_tf, int i, int j)
{
	LineSegmentDetection detection_tf_obj1(Eigen::Vector2d(detections_tf.at(0).x(), detections_tf.at(0).y()),
			                               Eigen::Vector2d(detections_tf.at(1).x(), detections_tf.at(1).y()),
										   ap_.get().getDetections()[i]->identifier());
	DalmrDashDetection detection_tf_obj2(Eigen::Vector2d(detections_tf.at(0).x(), detections_tf.at(0).y()),
			                             Eigen::Vector2d(detections_tf.at(1).x(), detections_tf.at(1).y()),
										 0.0, ap_.get().getDetections()[i]->identifier());

	float distance = 0.0;

	if (ap_.get().getLandmarks()[j]->individuallyCompatible(detection_tf_obj1)){
		distance = ap_.get().getLandmarks()[j]->distanceBetweenDetections(detection_tf_obj1);
	}else if (ap_.get().getLandmarks()[j]->individuallyCompatible(detection_tf_obj2)){
		distance = ap_.get().getLandmarks()[j]->distanceBetweenDetections(detection_tf_obj2);
	}

	return distance;
}

template<int Dim>
float SacBasedAssociation<Dim>::huberLoss (float error, float sigma){

	float loss;

	if (error <= sigma){
		loss = pow(error, 2.0) / 2.0;
	}else{
		loss = sigma * (error - sigma / 2.0);
	}

	return loss;
}

template<int Dim>
float SacBasedAssociation<Dim>::computeTotalError (Eigen::Isometry3d tf, bool save_asso)
{
	int N = ap_.get().getDetections().size();
	int M = ap_.get().getLandmarks().size();

	float error = 0;
	PairIds association;

    for (int i = 0; i < N; i++)
    {
    	float min_distance = 1000000;
    	association.first = i;
    	for (int j = 0; j < M; j++)
    	{
    		if (ap_.get().areIndividuallyCompatible(i, j)){
				float distance;
				std::vector<Eigen::Vector3d> detections_tf = ap_.get().getDetections()[i]->transformDetection(tf);
				if (detections_tf.size() == 1){
					distance = distanceForPoints (detections_tf, i, j);
				}else if (detections_tf.size() == 2){
					distance = distanceForLineSegments (detections_tf, i, j);
				}

				if (distance < min_distance)
				{
					min_distance = distance;
					association.second = j;
				}
    		}
    	}
    	error = error + min_distance;
    	if (save_asso && (min_distance < sac_cfg_.threshold_asso)){
    		addAssociation(association);
    	}
    }
	return error;
}

template<int Dim>
void SacBasedAssociation<Dim>::nnImplementation (void)
{
	Eigen::Isometry3d tf_null;
	tf_null.linear() = Eigen::Matrix<double, 3, 3>::Identity();
	tf_null.translation() = Eigen::Matrix<double, 3, 1>::Zero();

	computeTotalError (tf_null, true);
	return;
}

template<int Dim>
void SacBasedAssociation<Dim>::dcsacImplementation (Eigen::Isometry3d& tf_min)
{
	distanceCompatiblePairsSelection();

	// inicializations
	float error_min = 1000000;
	tf_min.linear() = Eigen::Matrix<double, 3, 3>::Identity();
	tf_min.translation() = Eigen::Matrix<double, 3, 1>::Zero();

	for (size_t i = 0; i < associations_candidates_.size(); i++){
		for (size_t j = 0; j < associations_candidates_[i].landmarks_pairs_ids.size(); j++){
			Eigen::Isometry3d tf;
			calculateTfBetweenPairs (i, j, tf);

			if (abs(tf(0, 3)) < sac_cfg_.x_var && abs(tf(1, 3)) < sac_cfg_.y_var && abs(acos(tf(0, 0))) < sac_cfg_.w_var)
			{
				float error = computeTotalError (tf, false);
				if (error < error_min)
				{
					error_min = error;
					tf_min = tf;
				}
			}
		}
	}

	float error = computeTotalError (tf_min, true);

	std::cout << "x: " << abs(tf_min(0, 3)) <<
			   ", y: " << abs(tf_min(1, 3)) <<
			   ", w: " << abs(acos(tf_min(0, 0))) << std::endl;

	std::cout << tf_min(0, 0) << ", " << tf_min(0, 1) << ", " << tf_min(0, 2) << ", " << tf_min(0, 3) << std::endl;
	std::cout << tf_min(1, 0) << ", " << tf_min(1, 1) << ", " << tf_min(1, 2) << ", " << tf_min(1, 3) << std::endl;
	std::cout << tf_min(2, 0) << ", " << tf_min(2, 1) << ", " << tf_min(2, 2) << ", " << tf_min(2, 3) << std::endl;
	std::cout << tf_min(3, 0) << ", " << tf_min(3, 1) << ", " << tf_min(3, 2) << ", " << tf_min(3, 3) << std::endl;

	std::cout << "Error: " << error_min << std::endl;

	return;
}

template<int Dim>
void SacBasedAssociation<Dim>::ransacImplementation (Eigen::Isometry3d& tf_min)
{
	randomSamplePairsSelection();

	float error_min = 1000000;

	int i = 0;
	while (i < associations_candidates_.size()){
		int j = 0;
		while (j < associations_candidates_[i].landmarks_pairs_ids.size()){
			Eigen::Isometry3d tf;
			calculateTfBetweenPairs (i, j, tf);
			float error = computeTotalError (tf, false);
			if (error < error_min)
			{
				error_min = error;
				tf_min = tf;
			}
			if (error_min < (sac_cfg_.error_threshold * ap_.get().getDetections().size()))
			{
				j = associations_candidates_[i].landmarks_pairs_ids.size();
				tf_min = tf;
			}
			j++;
		}
		if (error_min < (sac_cfg_.error_threshold * ap_.get().getDetections().size()))
		{
			i = associations_candidates_.size();
		}
		i++;
	}

	float error = computeTotalError (tf_min, true);

	return;
}

}
