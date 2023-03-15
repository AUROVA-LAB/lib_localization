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

}