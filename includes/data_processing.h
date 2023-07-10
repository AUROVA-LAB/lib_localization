#pragma once

#include "types_data.h"

namespace data_processing {

class DataProcessing {
public:
	DataProcessing(std::string config_path);
	DataProcessing(ConfigParams params);
	~DataProcessing() { }

	//// GET METHODS
	ConfigParams getParams (void){
		return params_;
	}
	PolylineMap getMap (void){
		return map_;
	}
	PolylineMap getLandmarks (void){
		return landmarks_;
	}
	PolylineMap getDetections (void){
		return detections_;
	}
	PointCloudPCL getLandmarksPcl (void){
		return landmarks_pcl_;
	}
	PointCloudPCL getDetectionsPcl (void){
		return detections_pcl_;
	}
	PointCloudPCL getAssociatedPcl (void){
		return associated_pcl_;
	}
	PointCloudPCL getCoregisteredPcl (void){
		return coregistered_pcl_;
	}
	IndexVector getLandmarksIndexI (void){
		return landmarks_i_;
	}
	IndexVector getLandmarksIndexJ (void){
		return landmarks_j_;
	}
	float getTranslationVarianceDaEvolution (void){
		float acum = 0.0;
		for (int i = 0; i < var_da_trs_.size(); i++) acum = acum + var_da_trs_.at(i);
		return (acum / var_da_trs_.size()) * params_.acum_tf_varfactor;
	}
	float getRotationVarianceDaEvolution (void){
		float acum = 0.0;
		for (int i = 0; i < var_da_rot_.size(); i++) acum = acum + var_da_rot_.at(i);
		return (acum / var_da_rot_.size()) * params_.acum_tf_varfactor;
	}


	//// SET METHODS
	void setDetections (PolylineMap detections){
		detections_ = detections;
	}
    void addTranslationDaEvolution (float tf_da_trs){
		var_da_trs_.push_back(tf_da_trs);
		if (var_da_trs_.size() > params_.acum_tf_da){
			var_da_trs_.erase(var_da_trs_.begin());
		}
	}
	void addRotationDaEvolution (float tf_da_rot){
		var_da_rot_.push_back(tf_da_rot);
		if (var_da_rot_.size() > params_.acum_tf_da){
			var_da_rot_.erase(var_da_rot_.begin());
		}
	}

	//// CLASS METHODS
	void readMapFromFile (void);
	float absdeltaAngleFromSegments (Segment segment_s1, Segment segment_s2);
	void sampleSegments (Segment segment, int id, float first_z, float second_z, Polyline& new_polyline);
	void samplePolylineMap (void);
	void createLandmarksFromMap (Pose2D position);
	void parseLandmarksToPcl (std::string frame);
	void parseDetectionsToPcl (std::string frame);
	void parseAssociationsToPcl (std::string frame, AssociationsVector& associations);
	void applyTfFromLandmarksToBaseFrame (Eigen::Isometry3d tf);
	void dataAssociationIcp (std::string frame, Eigen::Matrix4d& tf, AssociationsVector& associations);
	
private:
	ConfigParams params_;
	PolylineMap map_;
	PolylineMap landmarks_;
	PolylineMap detections_;

	PointCloudPCL landmarks_pcl_;
	PointCloudPCL detections_pcl_;
	PointCloudPCL coregistered_pcl_;
	PointCloudPCL associated_pcl_;

	IndexVector landmarks_i_;
	IndexVector landmarks_j_;

	std::vector<float> var_da_trs_;
	std::vector<float> var_da_rot_;
};

}