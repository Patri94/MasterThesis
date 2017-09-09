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
#include <trabajo/Marcador.hpp>

class CameraNode{
public:
    cv::Mat cam1,cam2,cam3,comb,ucomb,gray,thresholded;
    std::vector<KeyPoint> keyTag0,keyTag1,keyTag2,keyTag3,keyTag4,keyTag5;
    cv::Mat decrTag0,decrTag1,decrTag2,decrTag3,decrTag4,decrTag5;
    float brightnessAvg;
    int minContourPointsAllowed;
    int maxContourPointsAllowed;
    int minContourLengthAllowed;
    int maxContourLengthAllowed;
    vector <Mat> MarkersTemplate;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<Marcador> posiblesMarcas, Marcadores;
    sensor_msgs::ImagePtr ms1, ms2, ms3;
    std::vector<cv::Point2f> markerCorners;
    cv::Mat canonicalMarkerImage;
    cv::Size markerSize;
    std::vector<Marcador> OptMarkers;
    typedef const boost::function< void(const sensor_msgs::ImageConstPtr &)>  callback;
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg );
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg );
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg );
    void Init(void);
    void Concatenar(void);
    void brightnessAverage(void);
    void threshold(void);
    void LimpiarImagen(void);
    void Undistort(void);
    void findContours (void);
    void findCandidates(void);
    void Reconocer(void);
    ros::NodeHandle InitializeCallbacks(CameraNode Cam);
    int hammingDistance(cv::Mat bits);
    void recognizeMarkers(void);
    float linearInterpolator(const float& x, const cv::Point p1, const cv::Point p2);
    int MarkerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, Marcador &marker);
    void markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId);
    float perimeter(const std::vector<cv::Point2f> &a);
    cv::Mat rotate(cv::Mat input);
    void poseEstimation(void);


};
