#include "../includes/interface_ap.h"

namespace static_data_representation {

InterfaceAP::InterfaceAP(std::string config_path)
{
	// This is the option for load parameters from file (e.g. yaml).	
	return;
}

InterfaceAP::InterfaceAP(ConfigParams params)
{
	this->params_ = params;
	return;
}

void InterfaceAP::readMapFromFile (void)
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


float InterfaceAP::absdeltaAngleFromSegments (Segment segment_s1, Segment segment_s2)
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

void InterfaceAP::sampleSegments (Segment segment, int id, float first_z, float second_z, Polyline& new_polyline)
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
			//point.z = first_z + delta_z * i;
			if (i == 0) point.z = first_z;
			else if (i == sample_times-1) point.z = second_z;
			else point.z = 0.0;
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

void InterfaceAP::samplePolylineMap (void)
{
	PolylineMap map;
	map = map_;

	PolylineMap new_map;

	for(int i = 0; i < map.size(); i++){
		Polyline new_polyline;
		for(int j = 0; j < map.at(i).size() - 2; j++){

			Segment segment_s1, segment_s2;
			segment_s1.first = Eigen::Vector2d(map.at(i).at(j).x, map.at(i).at(j).y);
			segment_s1.second = Eigen::Vector2d(map.at(i).at(j+1).x, map.at(i).at(j+1).y);
			segment_s2.first = Eigen::Vector2d(map.at(i).at(j+1).x, map.at(i).at(j+1).y);
			segment_s2.second = Eigen::Vector2d(map.at(i).at(j+2).x, map.at(i).at(j+2).y);

			float delta_yaw = absdeltaAngleFromSegments (segment_s1, segment_s2);

			map.at(i).at(j+1).z = delta_yaw * params_.z_weight;

			sampleSegments (segment_s1, map.at(i).at(j+1).id, map.at(i).at(j).z, map.at(i).at(j+1).z, new_polyline);

			if (j == map.at(i).size() - 3){ // last iteration, to include last point
				segment_s1.first = Eigen::Vector2d(map.at(i).at(j+1).x, map.at(i).at(j+1).y);
				segment_s1.second = Eigen::Vector2d(map.at(i).at(j+2).x, map.at(i).at(j+2).y);
				segment_s2.first = Eigen::Vector2d(map.at(i).at(j+2).x, map.at(i).at(j+2).y);
				segment_s2.second = Eigen::Vector2d(map.at(i).at(j+2).x, map.at(i).at(j+2).y);

				sampleSegments (segment_s1, map.at(i).at(j+2).id, map.at(i).at(j+1).z, map.at(i).at(j+2).z, new_polyline);
				sampleSegments (segment_s2, map.at(i).at(j+2).id, map.at(i).at(j+2).z, map.at(i).at(j+2).z, new_polyline);
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

void InterfaceAP::createLandmarksFromMap (Pose2D position, float radious)
{
	landmarks_.clear();
	Polyline positions_way;

	for(int i = 0; i < map_.size(); i++){
		for(int j = 0; j < map_.at(i).size(); j++){
			float point_sf_x = map_.at(i).at(j).x - position.x;
			float point_sf_y = map_.at(i).at(j).y - position.y;
			float distance = sqrt(pow(point_sf_x, 2) + pow(point_sf_y, 2));

			if (distance < radious){
				positions_way.push_back(map_.at(i).at(j));
			}
		}
	}

	landmarks_.push_back(positions_way);
	return;
}

}