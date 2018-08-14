#include "opencv2/core/core.hpp"
#include "opencv2/aruco.hpp"


class MarkerGenerator{
	std::string path_;
public:
	MarkerGenerator();
	cv::Mat generate6x6Marker(int marker_id = 0, int marker_size = 200, int border_thickness = 1);
	void saveMarker(std::string name, cv::Mat marker_image);
};
