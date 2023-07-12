#pragma once

#include <iostream>
#include <vector>
#include <unistd.h>
#include <Eigen/Dense>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "latlong_utm.h"
#include "./rapidxml/rapidxml.hpp"
#include "./rapidxml/rapidxml_utils.hpp"

namespace data_processing {

struct PolylinePoint {
	double x;
	double y;
	double z;
	int id;
	int id_aux;
};

struct Pose2D {
	double x;
	double y;
	double w;
};

struct UtmToMapTrans {
	double x;
	double y;
};

struct ConfigParams {

	std::string url_to_map;
	std::string url_out_csv;

    float radious_dt;
	float radious_lm;

	int acum_tf_da;
	float acum_tf_varfactor;

	UtmToMapTrans utm2map_tr;

	int representation_type;

	float sample_distance;
	float z_weight;
	double compatibility_distance;

	float error_threshold;
	float x_var;
	float y_var;
	float w_var;
	float threshold_asso;
	float voxel_asso;

	// Sigmoid options for weights
	int type;
	float lambda;
	float k;
	float m;

	float odom_preweight;
};

using IndexVector = std::vector<int>;
using PointCloudPCL = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using Polyline = std::vector<PolylinePoint>;
using PolylineMap = std::vector<Polyline>;
using Trajectory = std::vector<Pose2D>;
using Segment = std::pair<Eigen::Vector2d, Eigen::Vector2d>;
using Tf = Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>;
using AssociationSingle = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using AssociationsVector = std::vector<AssociationSingle>;

} // namespace data_processing