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
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>




using namespace tf;


class CameraNode{
public:
    cv::Mat cam1,cam2,cam3,comb,ucomb,gray,thresholded,camMatrix;
    std::vector<KeyPoint> keyTag0,keyTag1,keyTag2,keyTag3,keyTag4,keyTag5;
    cv::Mat decrTag0,decrTag1,decrTag2,decrTag3,decrTag4,decrTag5;
    image_geometry::PinholeCameraModel PinModel;
    geometry_msgs::Pose EstimatedPose;
    float brightnessAvg;
    int minContourPointsAllowed;
    int maxContourPointsAllowed;
    int minContourLengthAllowed;
    int maxContourLengthAllowed;
    vector <Mat> MarkersTemplate;
    std::vector<Marcador> map;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<Marcador> posiblesMarcas, Marcadores;
    sensor_msgs::ImagePtr ms1, ms2, ms3;
    //sensor_msgs::Camera_Info caminfo;
    std::vector<cv::Point2f> markerCorners;
    cv::Mat canonicalMarkerImage;
    cv::Size markerSize;
    std::vector<Marcador> OptMarkers;
    ros::Publisher Publicar;
    ros::Publisher PublicarCam1,PublicarCam2,PublicarCam3;
    TransformBroadcaster br;
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
    //ros::NodeHandle InitializeCallbacks(CameraNode Cam);
    int hammingDistance(cv::Mat bits);
    void recognizeMarkers(void);
    float linearInterpolator(const float& x, const cv::Point p1, const cv::Point p2);
    int MarkerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, Marcador &marker);
    void markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId);
    float perimeter(const std::vector<cv::Point2f> &a);
    cv::Mat rotate(cv::Mat input);
    void poseEstimation(void);
    geometry_msgs::PoseArray LoadMap(void);
    int getMapSize(void);
    std::vector<cv::Point2d> Pinhole (std::vector<geometry_msgs::Pose> CamCoord);
    std::vector<geometry_msgs::Pose> ObservationModel (Marcador Marca, geometry_msgs::Pose CamaraMundo);
    void infoCallback(const sensor_msgs::ImageConstPtr&,const sensor_msgs::CameraInfoConstPtr& cam_info);
    float DirectTrans(double ox,double oy,double oz,double r,double p,double y);
    void InvTrans(double ox,double oy,double oz,double r,double p,double y);
    std::vector<Marcador> getMap(void);
    void PruebaMarca(Marcador Marca);
    void LoadParameters(void);
    cv::Point2d PinholeROS (geometry_msgs::Pose CamCoord);
    std::vector<cv::Point2d> Proyectar(geometry_msgs::PoseArray CamCoord1);
    void OdomCallback (const nav_msgs::OdometryConstPtr& msg);
    cv::Point2d Pinhole(cv::Point3d RelCoord);

};
