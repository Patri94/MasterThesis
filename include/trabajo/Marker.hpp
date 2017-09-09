#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/assign/list_of.hpp>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <functional>
#include <signal.h>

using namespace cv;
using namespace std;

class Marker{
	private:
		int etiqueta
                std::vector<cv::Point2f> Corners;
	public:
		void setCorners(std::vector<cv::Point2f> corners);
		void setEtiqueta(int etiqueta);
		std::vector<cv::Point2f> getCorners(void);
		int getEtiqueta(void);
};
