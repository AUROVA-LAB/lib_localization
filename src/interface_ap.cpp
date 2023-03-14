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
	Polyline positions_way;

	PolylinePoint position;
	position.x = 0.0;
	position.y = 0.0;
	position.z = 0.0;
	position.id = 0;

	positions_way.push_back(position);

	position.x = 4.0;
	position.y = 4.0;
	position.z = 0.0;
	position.id = 1;

	positions_way.push_back(position);

	position.x = 4.0;
	position.y = -4.0;
	position.z = 0.0;
	position.id = 2;

	positions_way.push_back(position);

	this->map_.push_back(positions_way);
  }
	return;
}

}