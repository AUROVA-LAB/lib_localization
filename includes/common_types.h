#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace static_data_representation {

struct PolylinePoint {
	double x;
	double y;
	double z;
	int id;
};

struct Pose2D {
	double x;
	double y;
	double w;
};

struct ConfigParams {

	std::string url_to_map;
	std::string url_out_csv;

	int representation_type;

	float sample_distance;
	float z_weight;
	double compatibility_distance;

	float error_threshold;
	float x_var;
	float y_var;
	float w_var;
	float threshold_asso;
};

using Polyline = std::vector<PolylinePoint>;
using PolylineMap = std::vector<Polyline>;
using Trajectory = std::vector<Pose2D>;
using Segment = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

}