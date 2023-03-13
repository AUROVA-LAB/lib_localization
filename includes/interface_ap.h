#include "common_types.h"

namespace static_data_representation {

class InterfaceAP {
public:
	InterfaceAP(std::string config_path);
	InterfaceAP(ConfigParams params);
	~InterfaceAP() { }
    
private:
	ConfigParams params_;
	PolylineMap map_landmarks_;
	PolylineMap map_detections_;
	Trajectory trajectory_;
};

}