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
#include <Marker.hpp>

void Marker::setCorners(std::vector<Point2f> corners){
	this->Corners=corners;
}

void Marker::setEtiqueta(int etiqueta){
	this->etiqueta=etiqueta;
}

std::vector<cv::Point2f> Marker::getCorners (void){
	return this->Corners;
}

int Marker::getEtiqueta(void){
	return this->etiqueta;
}

