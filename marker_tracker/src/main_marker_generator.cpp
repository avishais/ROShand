#include "marker_tracker/marker_generator.hpp"

int main(int argc, char** argv){
	MarkerGenerator mg;
	for (size_t i = 0; i < 6; i++){
		mg.saveMarker("marker" + std::to_string(i) + ".png",mg.generate6x6Marker(i,6*5));
	}
	return 0;
}
