#pragma once

#include "types_int.h"

namespace static_data_representation {

class InterfaceAP {
public:
	InterfaceAP(std::string config_path);
	InterfaceAP(ConfigParams params);
	~InterfaceAP() { }

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
	PointCloudPCL getCoregisteredPcl (void){
		return coregistered_pcl_;
	}
	IndexVector getLandmarksIndexI (void){
		return landmarks_i_;
	}
	IndexVector getLandmarksIndexJ (void){
		return landmarks_j_;
	}


	//// SET METHODS
	void setDetections (PolylineMap detections){
		detections_ = detections;
	}
    
	//// CLASS METHODS
	void readMapFromFile (void);
	float absdeltaAngleFromSegments (Segment segment_s1, Segment segment_s2);
	void sampleSegments (Segment segment, int id, float first_z, float second_z, Polyline& new_polyline);
	void samplePolylineMap (void);
	void createLandmarksFromMap (Pose2D position);
	void parseLandmarksToPcl (std::string frame);
	void parseDetectionsToPcl (std::string frame);
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

	IndexVector landmarks_i_;
	IndexVector landmarks_j_;
};

}