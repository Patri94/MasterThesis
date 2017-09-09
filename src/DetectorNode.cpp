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


#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75
#define CURVE_SIZE 4

using namespace cv;
using namespace std;
typedef const boost::function< void(const sensor_msgs::ImageConstPtr &)>  callback;
float perimeter(const std::vector<cv::Point2f> &a);
static void wrapper_imageCallback1(void* pt2Object, const sensor_msgs::ImageConstPtr& msg);



static void wrapper_imageCallback1(void* pt2Object, const sensor_msgs::ImageConstPtr& msg){
    CameraNode* mySelf = (CameraNode*) pt2Object;
    mySelf->imageCallback1(msg);
}
cv::Mat  output,camMat,distCoefs,comb;
vector<Mat> LoadCamParameters(void);


int main(int argc, char **argv){
 //Leer imagen publicada por el nodo.
      ros::init(argc, argv, "CameraNode");
      ros::NodeHandle nh;
      image_transport::ImageTransport it(nh);
      CameraNode CameraNodeobj;
     // ros::NodeHandle nhbis=CameraNodeobj.InitializeCallbacks(CameraNodeobj);
      callback boundImageCallback1 = boost::bind(&CameraNode::imageCallback1,&CameraNodeobj,_1);
      callback boundImageCallback2 = boost::bind(&CameraNode::imageCallback2,&CameraNodeobj,_1);
      callback boundImageCallback3 = boost::bind(&CameraNode::imageCallback3,&CameraNodeobj,_1);
      image_transport::Subscriber subcam1 = it.subscribe("Doris/camera1/image_raw", 1, boundImageCallback1);
      image_transport::Subscriber subcam2= it.subscribe("Doris/camera2/image_raw", 1, boundImageCallback2);
      image_transport::Subscriber subcam3 = it.subscribe("Doris/camera3/image_raw", 1, boundImageCallback3);
     // image_transport::Subscriber sub2 = it.subscribe("Doris/camera2/image_raw", 1, imageCallback2);
     // image_transport::Subscriber sub3 = it.subscribe("Doris/camera3/image_raw", 1, imageCallback3);
     // cv::namedWindow("view1");
     // cv::namedWindow("view2");
      //cv::namedWindow("view3");
      // cv::namedWindow("combi");


     // cv::startWindowThread();
       //Mat imagen1,imagen2,imagen3;
     // cout<<nhbis.ok()<<endl;
      CameraNodeobj.Init();
      while(nh.ok()){
              ros::spinOnce();
              //cout<<CameraNodeobj.cam1.size()<<endl;
              //cout<<CameraNodeobj.cam2.size()<<endl;
              //cout<<CameraNodeobj.cam3.size()<<endl;
              //imshow("view1",CameraNodeobj.cam1);
             // waitKey(30);
             //cout<<"uno"<<endl;
              cout<<"he llegado"<<endl;

              cout<<"Esquinas"<<CameraNodeobj.markerCorners.size()<<endl;
              CameraNodeobj.Concatenar();
              //CameraNodeobj.Undistort();
             // if (!(CameraNodeobj.comb.empty())){
                 // imshow("Undistorted",CameraNodeobj.comb);
                 // waitKey();
               // }
              //CameraNodeobj.brightnessAverage();
              CameraNodeobj.LimpiarImagen();
              CameraNodeobj.brightnessAverage();
              CameraNodeobj.threshold();
              //cv::waitKey()
              //cout<<"antes de contornos"<<CameraNodeobj.comb.size()<<endl;
             if (!(CameraNodeobj.comb.empty())){
                    CameraNodeobj.findContours();
                    cout<<"contornos"<<endl;
                    cout<<CameraNodeobj.contours.size()<<endl;
                    CameraNodeobj.findCandidates();
                    cout<<"Despues de findCandidates"<<CameraNodeobj.Marcadores.size()<<endl;
                     CameraNodeobj.OptMarkers.clear();
                     CameraNodeobj.recognizeMarkers();
              //CameraNodeobj.poseEstimation();*/


               }


          }
}

