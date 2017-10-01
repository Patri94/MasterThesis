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
//#include <trabajo/Marcador.hpp>
#include <trabajo/CameraNode.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75
#define CURVE_SIZE 4

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
typedef const boost::function< void(const sensor_msgs::ImageConstPtr &)>  callback;
typedef const boost::function< void(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&)> Callback;
typedef const boost::function< void(const nav_msgs::Odometry::ConstPtr& msg)> CallbackOdom;
float perimeter(const std::vector<cv::Point2f> &a);
static void wrapper_imageCallback1(void* pt2Object, const sensor_msgs::ImageConstPtr& msg);
cv::Mat  output,camMat,distCoefs,comb;
vector<Mat> LoadCamParameters(void);


int main(int argc, char **argv){
      ros::init(argc, argv, "CameraNode");
      ros::NodeHandle nh;
      ros::Publisher Publicar;
      ros::Publisher PublicarMapa;
      ros::Publisher Parameters;
      ros::Subscriber Odometry;
      image_transport::ImageTransport it(nh);
      CameraNode CameraNodeobj;
      callback boundImageCallback2 = boost::bind(&CameraNode::imageCallback2,&CameraNodeobj,_1);
      callback boundImageCallback3 = boost::bind(&CameraNode::imageCallback3,&CameraNodeobj,_1);
      CallbackOdom boundOdom = boost::bind(&CameraNode::OdomCallback,&CameraNodeobj,_1);
      image_transport::Subscriber subcam2= it.subscribe("Doris/camera2/image_raw", 1, boundImageCallback2);
      image_transport::Subscriber subcam3 = it.subscribe("Doris/camera3/image_raw", 1, boundImageCallback3);
      Odometry=nh.subscribe("/Doris/odom",1,boundOdom);
      message_filters::Subscriber<Image> image_sub(nh, "/Doris/camera1/image_raw", 1);
      message_filters::Subscriber<CameraInfo> info_sub(nh, "/Doris/camera1/camera_info", 1);
      TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
      sync.registerCallback(boost::bind(&CameraNode::infoCallback, &CameraNodeobj,_1, _2));
      Parameters=nh.advertise<sensor_msgs::CameraInfo> ("Parameters",1);
      CameraNodeobj.Publicar=nh.advertise<geometry_msgs::PoseArray> ("MarkerPose",1);
      CameraNodeobj.PublicarCam1=nh.advertise<geometry_msgs::PoseArray> ("MarkerPoseCam1",1);
      CameraNodeobj.PublicarCam2=nh.advertise<geometry_msgs::PoseArray> ("MarkerPoseCam2",1);
      CameraNodeobj.PublicarCam3=nh.advertise<geometry_msgs::PoseArray> ("MarkerPoseCam3",1);
      PublicarMapa=nh.advertise<geometry_msgs::PoseArray> ("Mapa",1);
      geometry_msgs::PoseArray PubMap;
      PubMap=CameraNodeobj.LoadMap();
      CameraNodeobj.Init();
      while(nh.ok()){
              //Incluir uana funcion task en Camera Node Obj que lo agrupe todo
              ros::spinOnce();
              PublicarMapa.publish(PubMap);
              CameraNodeobj.Concatenar();
              CameraNodeobj.LimpiarImagen();
              CameraNodeobj.brightnessAverage();
              CameraNodeobj.threshold();

             if (!(CameraNodeobj.comb.empty())){
                    CameraNodeobj.findContours();

                    CameraNodeobj.findCandidates();

                     CameraNodeobj.OptMarkers.clear();
                     CameraNodeobj.recognizeMarkers();
                     std::vector<Marcador> Mapa=CameraNodeobj.map;


                     for(int i=0;i<Mapa.size();i++){

                             CameraNodeobj.PruebaMarca(Mapa[i]);


              }
                 }




          }
}
