#include "../includes/data_processing.h"

namespace data_processing {

DataProcessing::DataProcessing(std::string config_path)
{
	// This is the option for load parameters from file (e.g. yaml).	
	return;
}

DataProcessing::DataProcessing(ConfigParams params)
{
	this->params_ = params;

	this->landmarks_pcl_ = PointCloudPCL(new pcl::PointCloud<pcl::PointXYZ>); 
	this->detections_pcl_ = PointCloudPCL(new pcl::PointCloud<pcl::PointXYZ>); 
	this->coregistered_pcl_ = PointCloudPCL(new pcl::PointCloud<pcl::PointXYZ>); 
	this->associated_dt_pcl_ = PointCloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
	this->associated_lm_pcl_ = PointCloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
	return;
}

void DataProcessing::readMapFromFile (void)
{
  // Check if the file exists
  if ((access(this->params_.url_to_map.c_str(), F_OK) != -1))
  {
    rapidxml::xml_document< > doc;
    rapidxml::xml_node< > *root_node;
    std::vector<double> coord;
    float lat, lon;
    long id_node, id_way;
    bool nextValue = true, nextValue2 = true;

    // Read the xml file into a vector
    std::ifstream theFile(this->params_.url_to_map.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');

    // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&buffer[0]);

	// Variables to convert ll to utm.
	Ellipsoid utm;
	double utm_x;
	double utm_y;
	char utm_zone[30];
	int ref_ellipsoid = 23;

	PolylinePoint position;
	Polyline positions_way;
	Polyline map_only_nodes;

	// Find our root node
    root_node = doc.first_node("osm");
    rapidxml::xml_node<> *node = root_node->first_node("node");

	// Load the nodes in "map_only_nodes".
    while (nextValue)
    {
		lat = atof(node->first_attribute("lat")->value());
		lon = atof(node->first_attribute("lon")->value());
		id_node = atof(node->first_attribute("id")->value());
		
		utm.LLtoUTM(ref_ellipsoid, lat, lon, utm_y, utm_x, utm_zone);

		position.x = utm_x - this->params_.utm2map_tr.x;
		position.y = utm_y - this->params_.utm2map_tr.y;
		position.z = 0.0;
		position.id = id_node;

		map_only_nodes.push_back(position);
		
	    if (!node->next_sibling("node")) nextValue = false;
		node = node->next_sibling();
    }

	nextValue = true;
    rapidxml::xml_node<> *way = root_node->first_node("way");

    // Load the links. The keyword 'way' is for links and the keyword 'ref' is for nodes that link
    while (nextValue)
    {
      id_way = atof(way->first_attribute("id")->value());
      
	  rapidxml::xml_node<> *nd = way->first_node("nd");
      nextValue2 = true;
	  positions_way.clear();

      // Connect all the nodes of a path
      while (nextValue2)
      {
		id_node = atof(nd->first_attribute("ref")->value());
		for (int k = 0; k < map_only_nodes.size(); k++){
			if (id_node == map_only_nodes.at(k).id){
				positions_way.push_back(map_only_nodes.at(k));
			}
		}

		if (!nd->next_sibling("nd")) nextValue2 = false;
		nd = nd->next_sibling();
      }

	  this->map_.push_back(positions_way);
	  
	  // Check if it is the last
      if (!way->next_sibling("way")) nextValue = false;
      way = way->next_sibling();

    }
    theFile.close();
  }
  return;
}


float DataProcessing::absdeltaAngleFromSegments (Segment segment_s1, Segment segment_s2)
{
	float delta_yaw = 0.0;

	Eigen::Vector2d s1_start = segment_s1.first;
	Eigen::Vector2d s1_end = segment_s1.second;
	Eigen::Vector2d s2_start = segment_s2.first;
	Eigen::Vector2d s2_end = segment_s2.second;

	float u1 = s1_end.x() - s1_start.x();
	float u2 = s1_end.y() - s1_start.y();

	float v1 = s2_end.x() - s2_start.x();
	float v2 = s2_end.y() - s2_start.y();

	float scalar_dot = u1 * v1 + u2 * v2;
	float u = sqrt(pow(u1, 2) + pow(u2, 2));
	float v = sqrt(pow(v1, 2) + pow(v2, 2));

	delta_yaw = acos(scalar_dot / (u * v));

	delta_yaw = abs(delta_yaw);

	return delta_yaw;
}

void DataProcessing::sampleSegments (Segment segment, int id, float first_z, float second_z, Polyline& new_polyline)
{
	PolylinePoint point;

	float dist_x = segment.second.x() - segment.first.x();
	float dist_y = segment.second.y() - segment.first.y();
	float dist_z = second_z - first_z;
	float distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2));

	int sample_times = std::floor(distance / params_.sample_distance);

	if (sample_times > 1){
		float delta_x = dist_x / sample_times;
		float delta_y = dist_y / sample_times;
		float delta_z = dist_z / sample_times;

		for (int i = 0; i < sample_times; i++){
			point.x = segment.first.x() + delta_x * i;
			point.y = segment.first.y() + delta_y * i;
			point.z = first_z + delta_z * i;
			if (i == 0) point.z = first_z;
			else if (i == sample_times-1) point.z = second_z;
			//else point.z = 0.0;
			point.id = id;

			new_polyline.push_back(point);
		}
	}else{
		point.x = segment.first.x();
		point.y = segment.first.y();
		point.z = first_z;
		point.id = id;

		new_polyline.push_back(point);
	}

	return;
}

void DataProcessing::samplePolylineMap (void)
{
	PolylineMap map;
	map = map_;

	PolylineMap new_map;

	for(int i = 0; i < map.size(); i++){
		Polyline new_polyline;
		Segment segment_s1, segment_s2;
		if((sqrt(pow(map.at(i).at(map.at(i).size()-1).x - map.at(i).at(0).x, 2) + // first iteration, to include z in point 0.
		         pow(map.at(i).at(map.at(i).size()-1).y - map.at(i).at(0).y, 2)) < params_.sample_distance) && (map.at(i).size() > 2)){
			segment_s1.first = Eigen::Vector2d(map.at(i).at(map.at(i).size()-2).x, map.at(i).at(map.at(i).size()-2).y);
			segment_s1.second = Eigen::Vector2d(map.at(i).at(0).x, map.at(i).at(0).y);
			segment_s2.first = Eigen::Vector2d(map.at(i).at(0).x, map.at(i).at(0).y);
			segment_s2.second = Eigen::Vector2d(map.at(i).at(1).x, map.at(i).at(1).y);

			float delta_yaw = absdeltaAngleFromSegments (segment_s1, segment_s2);
			map.at(i).at(0).z = delta_yaw * params_.z_weight;
		}else{
			map.at(i).at(0).z = 0.0;
		}
		for(int j = 0; j < map.at(i).size() - 2; j++){

			segment_s1.first = Eigen::Vector2d(map.at(i).at(j).x, map.at(i).at(j).y);
			segment_s1.second = Eigen::Vector2d(map.at(i).at(j+1).x, map.at(i).at(j+1).y);
			segment_s2.first = Eigen::Vector2d(map.at(i).at(j+1).x, map.at(i).at(j+1).y);
			segment_s2.second = Eigen::Vector2d(map.at(i).at(j+2).x, map.at(i).at(j+2).y);

			float delta_yaw = absdeltaAngleFromSegments (segment_s1, segment_s2);

			map.at(i).at(j+1).z = delta_yaw * params_.z_weight;

			sampleSegments (segment_s1, map.at(i).at(j+1).id, map.at(i).at(j).z, map.at(i).at(j+1).z, new_polyline);

			if (j == map.at(i).size() - 3){ // last iteration, to include last point
				sampleSegments (segment_s2, map.at(i).at(j+2).id, map.at(i).at(j+1).z, map.at(i).at(0).z, new_polyline);
			}
		}
		/*for (int k = 2; k < new_polyline.size(); k++){ // filter
			float new_z = (new_polyline.at(k-2).z + new_polyline.at(k-1).z + new_polyline.at(k).z) / 3;
			new_polyline.at(k).z = new_z;
		}*/
		if (new_polyline.size() > 1) new_map.push_back(new_polyline);
	}

	for (int i = 0; i < map_.size(); i++) map_.at(i).clear();
	map_.clear();
	map_ = new_map;

	return;
}

void DataProcessing::createLandmarksFromMap (Pose2D position)
{
	landmarks_.clear();
	Polyline positions_way;

	for(int i = 0; i < map_.size(); i++){
		for(int j = 0; j < map_.at(i).size(); j++){
			float point_sf_x = map_.at(i).at(j).x - position.x;
			float point_sf_y = map_.at(i).at(j).y - position.y;
			float distance = sqrt(pow(point_sf_x, 2) + pow(point_sf_y, 2));

			if (distance < params_.radious_lm){
				PolylinePoint point_tmp = map_.at(i).at(j);
				point_tmp.id = i;
				point_tmp.id_aux = j;
				positions_way.push_back(point_tmp);
			}
		}
	}

	landmarks_.push_back(positions_way);
	return;
}

void DataProcessing::parseLandmarksToPcl (std::string frame)
{
	this->landmarks_i_.clear();
	this->landmarks_j_.clear();
	this->landmarks_pcl_->clear();
	this->landmarks_pcl_->header.frame_id = frame;

	for (int i = 0; i < this->landmarks_.at(0).size(); i++){
		this->landmarks_pcl_->push_back(pcl::PointXYZ(this->landmarks_.at(0).at(i).x,
											          this->landmarks_.at(0).at(i).y, 0.0));
		this->landmarks_i_.push_back(this->landmarks_.at(0).at(i).id);
		this->landmarks_j_.push_back(this->landmarks_.at(0).at(i).id_aux);
	}
	return;
}

void DataProcessing::parseDetectionsToPcl (std::string frame)
{
	this->detections_pcl_->clear();
	this->detections_pcl_->header.frame_id = frame;

	for (int i = 0; i < this->detections_.at(0).size(); i++){
		this->detections_pcl_->push_back(pcl::PointXYZ(this->detections_.at(0).at(i).x,
											           this->detections_.at(0).at(i).y, 0.0));
	}
	return;
}

void DataProcessing::parseAssociationsLmToPcl (std::string frame, AssociationsVector& associations)
{
	this->associated_lm_pcl_->clear();
	this->associated_lm_pcl_->header.frame_id = frame;

	for (int i = 0; i < associations.size(); i++){
		this->associated_lm_pcl_->push_back(pcl::PointXYZ(associations.at(i).first.x(),
											              associations.at(i).first.y(), 0.0));
	}
	return;
}

void DataProcessing::parseAssociationsDtToPcl (std::string frame, AssociationsVector& associations)
{
	this->associated_dt_pcl_->clear();
	this->associated_dt_pcl_->header.frame_id = frame;

	for (int i = 0; i < associations.size(); i++){
		this->associated_dt_pcl_->push_back(pcl::PointXYZ(associations.at(i).second.x(),
											              associations.at(i).second.y(), 0.0));
	}
	return;
}

void DataProcessing::applyTfFromLandmarksToBaseFrame (Eigen::Isometry3d tf)
{
	for(int i = 0; i < landmarks_.size(); i++){
		for(int j = 0; j < landmarks_.at(i).size(); j++){
			Eigen::Vector3d point;
			point.x() = landmarks_.at(i).at(j).x;
			point.y() = landmarks_.at(i).at(j).y;
			point.z() = landmarks_.at(i).at(j).z;

			point = tf.inverse() * point;
			landmarks_.at(i).at(j).x = point.x();
			landmarks_.at(i).at(j).y = point.y();
			landmarks_.at(i).at(j).z = point.z();
		}
	}
	return;
}

void DataProcessing::dataAssociationIcp (std::string frame, Eigen::Matrix4d& tf, AssociationsVector& associations)
{
	//// DATA CO-REGISTRATION
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(this->detections_pcl_);
	icp.setInputTarget(this->landmarks_pcl_);

	this->coregistered_pcl_->clear();
	this->coregistered_pcl_->header.frame_id = frame;
	this->landmarks_z_.clear();


	icp.align(*this->coregistered_pcl_);

	tf = icp.getFinalTransformation().cast<double>();

	// Parse to affine to manage transforms
  	Eigen::Affine3d tf_aff(Eigen::Affine3f::Identity());
  	tf_aff.matrix() = tf;

	//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	//std::cout << tf << std::endl;

	//// DATA ASSOCIATION
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(this->coregistered_pcl_);
	vg.setLeafSize(params_.voxel_asso, params_.voxel_asso, 100.0f);
	vg.filter(*this->coregistered_pcl_);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  	kdtree.setInputCloud(this->landmarks_pcl_);
  	int K = 10;
	for (int i = 0; i < this->coregistered_pcl_->points.size(); i++){
      // Nearest point "landmarks_pcl"
      std::vector<int> pointIdxKNNSearch(K);
      std::vector<float> pointKNNSquaredDistance(K);
      pcl::PointXYZ search_point = this->coregistered_pcl_->points.at(i);
      if (kdtree.nearestKSearch (search_point, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0){
        if (sqrt(pointKNNSquaredDistance[0]) < params_.threshold_asso){ 

			AssociationSingle association;
          
			association.first = Eigen::Vector3d(map_.at(this->landmarks_i_.at(pointIdxKNNSearch[0])).at(this->landmarks_j_.at(pointIdxKNNSearch[0])).x,
                                                map_.at(this->landmarks_i_.at(pointIdxKNNSearch[0])).at(this->landmarks_j_.at(pointIdxKNNSearch[0])).y,
                                                0.0);

            association.second = tf_aff.inverse() * Eigen::Vector3d(this->coregistered_pcl_->points.at(i).x,
                                                                    this->coregistered_pcl_->points.at(i).y,
                                                                    0.0);
			associations.push_back(association);

			this->landmarks_z_.push_back(map_.at(this->landmarks_i_.at(pointIdxKNNSearch[0])).at(this->landmarks_j_.at(pointIdxKNNSearch[0])).z);

        }
      }
    }

	// Update DA evolution
	Eigen::Quaterniond tf_q(tf.block<3, 3>(0, 0));
	Eigen::Vector3d tf_p = tf.block<3, 1>(0, 3);
	Eigen::Vector3d tf_a = tf_q.toRotationMatrix().eulerAngles(0, 1, 2);
	double tf_yaw = abs(tf_a.z());
	double tf_dist = sqrt(pow(tf_p(0), 2) + pow(tf_p(1), 2));
	this->addRotationDaEvolution(tf_yaw);
	this->addTranslationDaEvolution(tf_dist);

	return;
}

float DataProcessing::dataInformation (void)
{
	float e = 2.7183;
	float acum = 0.0;
	float information = 0.0;
	float phi = 0.0;

	for (int i = 0; i < this->landmarks_z_.size(); i++) acum = acum + this->landmarks_z_.at(i);

	switch (params_.type){
		case 1:
			phi = (acum / params_.k) * params_.m - (params_.m / 2);
			break;
		case 2:
			phi = acum - params_.lambda;
			break;
		default:
			phi = acum - params_.lambda;
			break;
	}	
	information = 1 / (1 + pow(e, -phi));

	return information;
}

}