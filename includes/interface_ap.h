#include "types_int.h"
//#include <association_problem.h>

namespace static_data_representation {

class InterfaceAP {
public:

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
	void setDetections (PolylineMap detections){
		detections_ = detections;
	}

	InterfaceAP(std::string config_path);
	InterfaceAP(ConfigParams params);
	~InterfaceAP() { }
    
	void readMapFromFile (void);
	float absdeltaAngleFromSegments (Segment segment_s1, Segment segment_s2);
	void sampleSegments (Segment segment, int id, float first_z, float second_z, Polyline& new_polyline);
	void samplePolylineMap (void);
	void createLandmarksFromMap (Pose2D position, float radious);
	
private:
	ConfigParams params_;
	PolylineMap map_;
	PolylineMap landmarks_;
	PolylineMap detections_;
};

}