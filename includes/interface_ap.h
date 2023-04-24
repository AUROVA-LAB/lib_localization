#include "types_int.h"

namespace static_data_representation {

class InterfaceAP {
public:

	PolylineMap getMap (void){
		return map_;
	}

	InterfaceAP(std::string config_path);
	InterfaceAP(ConfigParams params);
	~InterfaceAP() { }
    
	void readMapFromFile (void);
	
private:
	ConfigParams params_;
	PolylineMap map_;
};

}