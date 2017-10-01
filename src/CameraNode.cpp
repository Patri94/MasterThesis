#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
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
#include <trabajo/CameraNode.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <list>
#include <numeric>
#include <vector>
#include <string>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/geometry/geometries/adapted/boost_array.hpp>

#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75
#define CELL_MARKER_SIZE 7
#define CURVE_SIZE 4
#define IMAGE_WIDTH 1812
#define MARKER_HEIGHT 0.42// 0.545985
#define MARKER_WIDTH 0.297 //0.3777
#define NUM_CAM 3


using namespace cv;
using namespace std;
using namespace tf2;
using namespace nav_msgs;

void CameraNode::Init(void){

    markerSize = cv::Size(215, 345);

    this->markerCorners.push_back(cv::Point2f(0, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, markerSize.height - 1));
    this->markerCorners.push_back(cv::Point2f(0, markerSize.height - 1));

}

geometry_msgs::PoseArray CameraNode::LoadMap(void){
    
    //Cuando todo funcione correctamente pasar a un fichero y leerlo de ahi.
    FileStorage fs ("Mapa.yaml",FileStorage::WRITE);
    const int IDs[]={17,18,19,20,21,22};
    const float arr0[]={-2.2749,2.5,2.5,-M_PI/2,0,-1.57,17};
    const float arr1[]={-2.2749,-0.8,2.5,-M_PI/2,0,-1.57,18};
    const float arr2[]={0.707,-4.525,2.5,M_PI/2,M_PI,3.14,19};
    const float arr3[]={3.85972,-0.8,2.5,-M_PI/2,0,1.57,20};
    const float arr4[]={3.85972,2.5,2.5,-M_PI/2,0,1.57,21};
    const float arr5[]={0.707,6.675,2.5,-M_PI/2,0,3.14,22};

    write (fs,"IDs",IDs);
    write (fs,"arr0",arr0);
    write (fs,"arr1",arr1);
    write (fs,"arr2",arr2);
    write (fs,"arr3",arr3);
    write (fs,"arr4",arr4);
    write (fs,"arr5",arr5);


    std::vector<float> pos0(arr0,arr0 +(sizeof(arr0)/sizeof(arr0[0])));
    std::vector<float> pos1(arr1,arr1 +(sizeof(arr1)/sizeof(arr1[0])));
    std::vector<float> pos2(arr2,arr2 +(sizeof(arr2)/sizeof(arr2[0])));
    std::vector<float> pos3(arr3,arr3 +(sizeof(arr3)/sizeof(arr3[0])));
    std::vector<float> pos4(arr4,arr4 +(sizeof(arr4)/sizeof(arr4[0])));
    std::vector<float> pos5(arr5,arr5 +(sizeof(arr5)/sizeof(arr5[0])));

    std::vector<vector <float> > Poses;
    std::vector<Marcador> mapa;
    
    Poses.push_back(pos0);
    Poses.push_back(pos1);
    Poses.push_back(pos2);
    Poses.push_back(pos3);
    Poses.push_back(pos4);
    Poses.push_back(pos5);

    geometry_msgs::Pose LeftUpCorner,RightUpCorner,RightDownCorner,LeftDownCorner;
    tf::Transform LeftUp,LeftDown,RightUp,RightDown;
    tf::Matrix3x3 RotCorner;
    geometry_msgs::Quaternion QuatCorner;
    tf::Quaternion QuaternionTF;
    QuatCorner=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    //LeftUpCorner
    LeftUpCorner.position.x=MARKER_WIDTH/2;
    LeftUpCorner.position.y=-MARKER_HEIGHT/2;
    LeftUpCorner.position.z=0;
    LeftUpCorner.orientation=QuatCorner;
    LeftUp.setOrigin(tf::Vector3(LeftUpCorner.position.x, LeftUpCorner.position.y, LeftUpCorner.position.z));
    quaternionMsgToTF(QuatCorner,QuaternionTF);
    LeftUp.setRotation(QuaternionTF);
    //LeftDownCorner
    LeftDownCorner.position.x= MARKER_WIDTH/2;
    LeftDownCorner.position.y=MARKER_HEIGHT/2;
    LeftDownCorner.position.z=0;
    LeftDownCorner.orientation=QuatCorner;
    LeftDown.setOrigin(tf::Vector3(LeftDownCorner.position.x, LeftDownCorner.position.y, LeftDownCorner.position.z));
    LeftDown.setRotation(QuaternionTF);
    //RightUpCorner
    RightUpCorner.position.x= -MARKER_WIDTH/2;
    RightUpCorner.position.y=-MARKER_HEIGHT/2;
    RightUpCorner.position.z=0;
    RightUpCorner.orientation=QuatCorner;
    RightUp.setOrigin(tf::Vector3(RightUpCorner.position.x, RightUpCorner.position.y, RightUpCorner.position.z));
    RightUp.setRotation(QuaternionTF);
    //RightDownCorner
    RightDownCorner.position.x= -MARKER_WIDTH/2;
    RightDownCorner.position.y=MARKER_HEIGHT/2;
    RightDownCorner.position.z=0;
    RightDownCorner.orientation=QuatCorner;
    RightDown.setOrigin(tf::Vector3(RightDownCorner.position.x, RightDownCorner.position.y, RightDownCorner.position.z));
    RightDown.setRotation(QuaternionTF);

   // write (fs,"LeftUp",LeftUp);
    //write (fs,"LeftDown",LeftDown);
   // write (fs,"LeftUp",RightUp);
   // write (fs,"LeftUp",RightDown);
    //Transformation World to Center for every Corner.
    geometry_msgs::PoseArray PubMap;
    PubMap.header.frame_id="ground_plane__link";
    std::vector<tf::Transform> TransMundo;
    std::vector<geometry_msgs::Pose> Centros;
    for (int j=0;j<6;j++){
            tf::Transform MundoCenter;
            //Marcador Marker;
           // Marker.setMarkerId(IDs[j]);
            geometry_msgs::Pose PoseMarker;
            tf::Matrix3x3 orientation;
            tf::Quaternion Quat;
            PoseMarker.position.x = Poses[j][0];
            PoseMarker.position.y = Poses[j][1];
            PoseMarker.position.z = Poses[j][2];
            orientation.setRPY (Poses[j][3],Poses[j][4],Poses[j][5]);
            orientation.getRotation(Quat);
            PoseMarker.orientation.x = double(Quat.x());
            PoseMarker.orientation.y = double(Quat.y());
            PoseMarker.orientation.z = double(Quat.z());
            PoseMarker.orientation.w = double(Quat.w());
            tf::Vector3 TraslMundo(PoseMarker.position.x,PoseMarker.position.y,PoseMarker.position.z);
            MundoCenter.setOrigin(TraslMundo);
            MundoCenter.setRotation(Quat);
            TransMundo.push_back(MundoCenter);
            PubMap.poses.push_back(PoseMarker);
            Centros.push_back(PoseMarker);

        }
   // write(fs,"Centros",Centros);
    //write(fs,"TransMundo",TransMundo);

    //Corners in World Coordinates
    vector<Marcador> Mapa;
    std::vector<std::vector<geometry_msgs::Pose> > MarkersCornersPose;
    std::vector<std::vector<geometry_msgs::Transform> > MarkerCornersTf;
    for (int i=0;i<6;i++){
            std::vector<geometry_msgs::Pose> CornersPoseWorld;
            Marcador Marca;
            Marca.setMarkerId(IDs[i]);
            Marca.setCenterWorld(Centros[i]);
            tf::Transform DefLeftUp,DefLeftDown,DefRightUp,DefRightDown;
            geometry_msgs::Transform LeftUpMsg,LeftDownMsg,RightUpMsg,RightDownMsg;
            geometry_msgs::TransformStamped LeftUpMsgSt,LeftDownMsgSt,RightUpMsgSt,RightDownMsgSt;
            geometry_msgs::Transform DefLeftUpMsg,DefLeftDownMsg,DefRightUpMsg,DefRightDownMsg;
            geometry_msgs::PoseStamped StampedLeftUp, StampedLeftDown,StampedRightUp,StampedRightDown,LeftUp,LeftDown,RightUp,RightDown;
            StampedLeftUp.pose.position=LeftUpCorner.position;
            StampedLeftUp.pose.orientation=LeftUpCorner.orientation;
            StampedLeftDown.pose.position=LeftDownCorner.position;
            StampedLeftDown.pose.orientation=LeftDownCorner.orientation;
            StampedRightUp.pose.position=RightUpCorner.position;
            StampedRightUp.pose.orientation=RightUpCorner.orientation;
            StampedRightDown.pose.position=RightDownCorner.position;
            StampedRightDown.pose.orientation=RightDownCorner.orientation;
            geometry_msgs::TransformStamped StDefLeftUp,StDefLeftDown,StDefRightUp,StDefRightDown;
            DefLeftUp=TransMundo[i];
            DefLeftDown=TransMundo[i];
            DefRightUp=TransMundo[i];
            DefRightDown=TransMundo[i];
            tf::Vector3 origin=DefLeftUp.getOrigin();
            tf::Quaternion Quatr = DefLeftDown.getRotation();
            transformTFToMsg(DefLeftUp,DefLeftUpMsg);
            transformTFToMsg(DefLeftDown,DefLeftDownMsg);
            transformTFToMsg(DefRightDown,DefRightDownMsg);
            transformTFToMsg(DefRightUp,DefRightUpMsg);
            StDefLeftUp.transform = DefLeftUpMsg;
            StDefLeftDown.transform = DefLeftDownMsg;
            StDefRightUp.transform = DefRightUpMsg;
            StDefRightDown.transform = DefRightDownMsg;
            tf2::doTransform(StampedLeftUp,LeftUp,StDefLeftUp);
            tf2::doTransform(StampedLeftDown,LeftDown,StDefLeftDown);
            tf2::doTransform(StampedRightUp,RightUp,StDefRightUp);
            tf2::doTransform(StampedRightDown,RightDown,StDefRightDown);
            //CounterClockwise
            CornersPoseWorld.push_back(LeftUp.pose);
            CornersPoseWorld.push_back(LeftDown.pose);
            CornersPoseWorld.push_back(RightDown.pose);
            CornersPoseWorld.push_back(RightUp.pose);
            Marca.setPoseWorld(CornersPoseWorld);
            std::vector<geometry_msgs::Transform> CornersTf;
            CornersTf.push_back(DefLeftUpMsg);
            CornersTf.push_back(DefLeftDownMsg);
            CornersTf.push_back(DefRightDownMsg);
            CornersTf.push_back(DefRightUpMsg);
            Marca.setTransformCorners(CornersTf);
            Mapa.push_back(Marca);
            PubMap.poses.push_back(LeftUp.pose);
            PubMap.poses.push_back(LeftDown.pose);
            PubMap.poses.push_back(RightUp.pose);
            PubMap.poses.push_back(RightDown.pose);
            MarkersCornersPose.push_back(CornersPosWorld);
            MarkerCornersTf.push_back(CornersTf);



        }
    //write(fs,"MarkersCornersPose",TransMundo);
    //write(fs,"MarkersCornersTf",MarkerCornersTf);
    this->map=Mapa;
    cout<<"Guardado"<<endl;
    cv::waitKey();


    return PubMap;



}


std::vector<cv::Point2d> CameraNode::Proyectar(geometry_msgs::PoseArray CamCoord1){
    //TF to CAM1 (Center)
    geometry_msgs::Pose Cam1;
    Cam1.position.x=0.0;
    Cam1.position.y=0.0;
    Cam1.position.z=0.1;
    geometry_msgs::Quaternion QuatCam1;
    QuatCam1=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    Cam1.orientation=QuatCam1;
    tf::Quaternion QuatT;
    QuatT.setRPY(0,0,0);
    geometry_msgs::PoseStamped Cam1St;
    tf::Transform CamTCam1,invCamTCam1;
    geometry_msgs::TransformStamped invCamTCam1St,CamTCam1St;
    tf::Vector3 Trasl (Cam1.position.x,Cam1.position.y,Cam1.position.z);
    CamTCam1.setOrigin(Trasl);
    CamTCam1.setRotation(QuatT);
    invCamTCam1=CamTCam1.inverse();
    transformTFToMsg(CamTCam1,CamTCam1St.transform);
    transformTFToMsg(invCamTCam1,invCamTCam1St.transform);
    CamTCam1St.header.frame_id="camera_link";
    CamTCam1St.child_frame_id="Cam1";
    this->br.sendTransform(CamTCam1St);
    //TF to CAM2 (Left)
    geometry_msgs::Pose Cam2;
    Cam2.position.x=-0.08660254;
    Cam2.position.y=0.0;
    Cam2.position.z=-0.05;
    geometry_msgs::Quaternion QuatCam2;
    QuatCam2=tf::createQuaternionMsgFromRollPitchYaw(0,-2.093,0);
    Cam2.orientation=QuatCam2;
    tf::Quaternion QuatT2;
    QuatT2.setRPY(0,-2.093,0);
    geometry_msgs::PoseStamped Cam2St;
    tf::Transform CamTCam2,invCamTCam2;
    geometry_msgs::TransformStamped invCamTCam2St,CamTCam2St;
    tf::Vector3 Trasl2 (Cam2.position.x,Cam2.position.y,Cam2.position.z);
    CamTCam2.setOrigin(Trasl2);
    CamTCam2.setRotation(QuatT2);
    invCamTCam2=CamTCam2.inverse();
    transformTFToMsg(CamTCam2,CamTCam2St.transform);
    transformTFToMsg(invCamTCam2,invCamTCam2St.transform);
    CamTCam2St.header.frame_id="camera_link";
    CamTCam2St.child_frame_id="Cam2";
    this->br.sendTransform(CamTCam2St);
    //TF a Cam3 (Right)
    geometry_msgs::Pose Cam3;
    Cam3.position.x=0.08660254;
    Cam3.position.y=0.0;
    Cam3.position.z=-0.05;
    geometry_msgs::Quaternion QuatCam3;
    QuatCam3=tf::createQuaternionMsgFromRollPitchYaw(0,2.093,0);
    Cam3.orientation=QuatCam3;
    tf::Quaternion QuatT3;
    QuatT3.setRPY(0,2.093,0);
    geometry_msgs::PoseStamped Cam3St;
    tf::Transform CamTCam3,invCamTCam3;
    geometry_msgs::TransformStamped invCamTCam3St,CamTCam3St;
    tf::Vector3 Trasl3 (Cam3.position.x,Cam3.position.y,Cam3.position.z);
    CamTCam3.setOrigin(Trasl3);
    CamTCam3.setRotation(QuatT3);
    invCamTCam3=CamTCam3.inverse();
    transformTFToMsg(CamTCam3,CamTCam3St.transform);
    transformTFToMsg(invCamTCam3,invCamTCam3St.transform);
    CamTCam3St.header.frame_id="camera_link";
    CamTCam3St.child_frame_id="Cam3";
    this->br.sendTransform(CamTCam3St);

    //Corners in CAM1 Coordinates
    geometry_msgs::PoseArray MarkersCam1;
    MarkersCam1.header.frame_id="Cam1";
    for (int i=0;i<CamCoord1.poses.size();i++){
            geometry_msgs::PoseStamped CoordCam1St,CoordCamTransSt;
            CoordCam1St.pose=CamCoord1.poses[i];
            tf2::doTransform(CoordCam1St,CoordCamTransSt,invCamTCam1St);
            MarkersCam1.poses.push_back(CoordCamTransSt.pose);
        }
    this->PublicarCam1.publish(MarkersCam1);
    //Corners in CAM2 Coordinates
    geometry_msgs::PoseArray MarkersCam2;
    MarkersCam2.header.frame_id="Cam2";
    for (int i=0;i<CamCoord1.poses.size();i++){
            geometry_msgs::PoseStamped CoordCam2St,CoordCamTransSt2;
            CoordCam2St.pose=CamCoord1.poses[i];
            tf2::doTransform(CoordCam2St,CoordCamTransSt2,invCamTCam2St);
            MarkersCam2.poses.push_back(CoordCamTransSt2.pose);
        }
    this->PublicarCam2.publish(MarkersCam2);
    //Corners in CAM3 Coordinates
    geometry_msgs::PoseArray MarkersCam3;
    MarkersCam3.header.frame_id="Cam3";
    for (int i=0;i<CamCoord1.poses.size();i++){
            geometry_msgs::PoseStamped CoordCam3St,CoordCamTransSt3;
            CoordCam3St.pose=CamCoord1.poses[i];
            tf2::doTransform(CoordCam3St,CoordCamTransSt3,invCamTCam3St);
            MarkersCam3.poses.push_back(CoordCamTransSt3.pose);
        }
    this->PublicarCam3.publish(MarkersCam3);

    //Which Camera?
    std::vector<cv::Point2d> Pixels;
    for (int i=0;i<CamCoord1.poses.size();i++){
     //Angle respect to Z of CAM1
    float angulo;
    angulo = atan2(double(CamCoord1.poses[i].position.x),double(CamCoord1.poses[i].position.z));
    if (angulo<0){
            angulo=angulo+(2*M_PI);
        }
    cv::Point2d Pixel,offset,Pixel2;
    offset.x=IMAGE_WIDTH/NUM_CAM;
    offset.y=0;
    cv::Point3d Coord;
    //CAM2
    if (angulo>M_PI and angulo<5.2333){
            Coord.x=MarkersCam2.poses[i].position.x;
            Coord.y=MarkersCam2.poses[i].position.y;
            Coord.z=MarkersCam2.poses[i].position.z;
            Pixel=this->PinModel.project3dToPixel(Coord);
            Pixel=Pixel-offset;

        }else{
            //CAM3
            if(angulo>1.047 and angulo<M_PI){
                    Coord.x=MarkersCam3.poses[i].position.x;
                    Coord.y=MarkersCam3.poses[i].position.y;
                    Coord.z=MarkersCam3.poses[i].position.z;
                    Pixel=this->PinModel.project3dToPixel(Coord);
                    Pixel=Pixel+offset;
                }else{//CAM1
                    Coord.x=MarkersCam1.poses[i].position.x;
                    Coord.y=MarkersCam1.poses[i].position.y;
                    Coord.z=MarkersCam1.poses[i].position.z;
                    Pixel=this->PinModel.project3dToPixel(Coord);
                }
        }
     Pixels.push_back(Pixel);
        }

    return Pixels;

}


void CameraNode::OdomCallback (const nav_msgs::Odometry::ConstPtr& msg){
    this->EstimatedPose=msg->pose.pose;
}

//Relative Pose to the Center of Camera Link
std::vector<geometry_msgs::Pose> CameraNode::ObservationModel (Marcador Marca, geometry_msgs::Pose CamaraMundo){
    //Pose CAM;
    tf::Transform MundTrob, invMundTrob,RobTCam,invRobotTCam;
    tf::Quaternion RotCam;
    //From Robot base to camera
    RotCam.setRPY(-M_PI/2,0,-M_PI/2);//Pich de M_PI/2
    RobTCam.setOrigin(tf::Vector3(0,0,1.4));
    RobTCam.setRotation(RotCam);
    tf::Quaternion QMundRCam (CamaraMundo.orientation.x,CamaraMundo.orientation.y,CamaraMundo.orientation.z,CamaraMundo.orientation.w);
    tf::Vector3 Trasl1 (CamaraMundo.position.x,CamaraMundo.position.y,CamaraMundo.position.z);
    //From World to Robot
    MundTrob.setRotation(QMundRCam);
    MundTrob.setOrigin(Trasl1);
    //Inverse the transformation--> inversa del mundo a la camara
    invRobotTCam=RobTCam.inverse();
    invMundTrob = MundTrob.inverse();
    geometry_msgs::TransformStamped MundTrobSt, RobotTCamSt;
    MundTrobSt.header.frame_id="ground_plane__link";
    MundTrobSt.child_frame_id="EstimatedPose";
    RobotTCamSt.header.frame_id="EstimatedPose";
    RobotTCamSt.child_frame_id="camera_link";
    transformTFToMsg(MundTrob,MundTrobSt.transform);
    transformTFToMsg(RobTCam,RobotTCamSt.transform);
    this->br.sendTransform(MundTrobSt);
    this->br.sendTransform(RobotTCamSt);

    //Pose Transformation
    geometry_msgs::TransformStamped invMundTrobStamped,invRobotTCamSt;
    transformTFToMsg(invMundTrob,invMundTrobStamped.transform);
    transformTFToMsg(invRobotTCam,invRobotTCamSt.transform);
    std::vector<geometry_msgs::Pose> RelativaCorners,PoseWorld;
    std::vector<geometry_msgs::Transform> Corners = Marca.getTransformCorners();
    PoseWorld=Marca.getPoseWorld();
    for (int i=0;i<4;i++){
            tf::Transform CornerDef,CornersTf;
            transformMsgToTF(Corners[i],CornersTf);
            geometry_msgs::PoseStamped CornerPose,WorldPose,Inter2;
            geometry_msgs::TransformStamped CornerSt;
            WorldPose.pose=PoseWorld[i];
             transformTFToMsg(CornerDef,CornerSt.transform);
             tf2::doTransform(WorldPose,Inter2,invMundTrobStamped);
             tf2::doTransform(Inter2,CornerPose,invRobotTCamSt);
            RelativaCorners.push_back(CornerPose.pose);

        }
    //cout<<"Tengo la posicion relativa"<<endl;
    return RelativaCorners;



}

void CameraNode::PruebaMarca(Marcador Marca){
    geometry_msgs::Pose Supuesta,Supuesta2;
    Supuesta.position.x=0;
    Supuesta.position.y=0;
    Supuesta.position.z=0;
    tf::Quaternion Quat;
    tf::Matrix3x3 Mat;
    geometry_msgs::Quaternion QuatMs;
    Mat.setRPY(0,0,0);
    Mat.getRotation(Quat);
    tf::quaternionTFToMsg (Quat,QuatMs);
    Supuesta.orientation=QuatMs;
    std::vector<cv::Point2d> proyeccion;
    std::vector<geometry_msgs::Pose> Relative=this->ObservationModel(Marca,Supuesta);
    Marca.setRelativePose(Relative);
    geometry_msgs::PoseArray CornersRelativePose;
    CornersRelativePose.header.frame_id="camera_link";
    for (int i=0;i<4;i++){
            geometry_msgs::Pose position = Relative[i];
            geometry_msgs::PoseStamped msg;
            msg.header.frame_id="camera_link";
            msg.pose=Relative[i];
            CornersRelativePose.poses.push_back(Relative[i]);
        }
   this->Publicar.publish(CornersRelativePose);
   proyeccion=Proyectar(CornersRelativePose);

    //Dibujar situacion estimada
   line (this->comb,proyeccion[0], proyeccion[1],Scalar(0,0,255),1);
   line (this->comb,proyeccion[1], proyeccion[2],Scalar(0,0,255),1);
   line (this->comb,proyeccion[2], proyeccion[3],Scalar(0,0,255),1);
   line (this->comb,proyeccion[3], proyeccion[0],Scalar(0,0,255),1);
   int ID=Marca.getMarkerID();
   cout<<"Etiqueta"<<ID<<endl;
   std::string etiqueta ;
   ostringstream convert;
   convert<<ID;
   etiqueta=convert.str();
   putText(this->comb, etiqueta, proyeccion[0],CV_FONT_HERSHEY_COMPLEX,0.8,Scalar(0,0,255));

   imshow("Prueba",comb);
   waitKey(30);

}

std::vector<Marcador> CameraNode::getMap(void){
    return this->map;
}

//SUBSCRIBERS TO IMAGE TOPICS
void CameraNode::infoCallback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info){
    sensor_msgs::CameraInfo cam_inf_ed;
    cam_inf_ed.header.frame_id="Cam1";
    cam_inf_ed.height=679;
    cam_inf_ed.width=604;
    cam_inf_ed.distortion_model="plumb_bob";
    double Da[5]={-0.2601958609577983, 0.05505240192232372, 0.0, -0.0045449850126361765, 0.0};
    boost::array<double, 9ul> K={ {174.746839097, 0.0, 906.0, 0.0, 174.746839097, 339.5, 0.0, 0.0, 1.0} } ;
    boost::array<double, 9ul> R={ {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0} };
    boost::array<double, 12ul> P={ {174.64077512103418, 0.0, 906.0, 0.0, 0.0, 174.64077512103418, 339.5, 0.0, 0.0, 0.0, 1.0, 0.0} };
    std::vector<double> D(Da,Da +(sizeof(Da)/sizeof(Da[0])));
    cam_inf_ed.D=D;
    cam_inf_ed.K=K;
    cam_inf_ed.R=R;
    cam_inf_ed.P=P;
    cam_inf_ed.binning_x=0.0;
    cam_inf_ed.binning_y=0.0;
    cam_inf_ed.roi.height=0;
    cam_inf_ed.roi.width=0;

    this->PinModel.fromCameraInfo(cam_inf_ed);
    this->cam1 = cv_bridge::toCvShare(msg, "bgr8")->image;
     this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam1).toImageMsg();
}


void CameraNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg){
    this->cam2 = cv_bridge::toCvShare(msg, "bgr8")->image;
    this->ms2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam2).toImageMsg();
}

void CameraNode::imageCallback3(const sensor_msgs::ImageConstPtr& msg){
    this->cam3 = cv_bridge::toCvShare(msg, "bgr8")->image;
    this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam3).toImageMsg();
}
void CameraNode::Concatenar(void){
    if (!(this->cam1.empty()) && !(this->cam2.empty()) && !(this->cam3.empty())){
    cv::hconcat(this->cam2,this->cam1,this->comb);
    cv::hconcat(this->comb,this->cam3,this->comb);
    cv::imshow("combi", this->comb);
    cv::waitKey(30);
        }
}

//DETECTOR DE BALIZAS VISUALES
void CameraNode::LimpiarImagen(void){
    if(!(this->comb.empty())){
    int erosionSize = 0;
    cv::Mat erosion;
    cv::Mat element=cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize - 1, erosionSize - 1));
    cv::erode(this->comb, erosion, element);
    cv::dilate(erosion, this->comb, element);
    cv::imshow("limpia", this->comb);
    cv::waitKey(30);
        }
}
void CameraNode::brightnessAverage(void){
    if(!(this->comb.empty())){
    cv::cvtColor(this->comb, this->gray, CV_BGR2GRAY);
    int histSize = 256;
    float range[] = { 0,256 };
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;
    cv::Mat hist;
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0) );
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    for( int i = 1; i < histSize; i++ ){
                    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                            cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                            cv::Scalar( 0, 0, 255), 2, 8, 0  );
            }
    this->brightnessAvg=0;

            for(int i =0; i< 256; i++){
                this->brightnessAvg+= hist.at<float>(i)*i/cv::sum(hist)[0];  //Media ponderada
            }

    //cout<<this->brightnessAvg<<endl;
        }
}

void CameraNode::threshold(void){
    if(!(this->comb.empty())){
    cv::cvtColor(this->comb,this->gray, CV_BGR2GRAY);
    float index= 75.0 / 255.0 * this->brightnessAvg ;
    cv::adaptiveThreshold(this->gray,this->thresholded, 255,  CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD, (int)index);
        }
}
void CameraNode::findContours (void){
    RNG rng(12345);
    this->minContourPointsAllowed=35;
    this->maxContourPointsAllowed=70;
    this->contours.clear();
    cv::Mat edges;
     vector<Vec4i> hierarchy;
     std::vector<std::vector<cv::Point> > Todos;
    cv::Canny(this->thresholded,edges,100,180,5);
    cv::findContours (edges,Todos,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for (unsigned int i = 0; i < Todos.size(); i++){
                    if (Todos.at(i).size() > minContourPointsAllowed){
                            contours.push_back(Todos.at(i));
                    }
    }
      Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
      for( int i = 0; i< contours.size(); i++ )
         {
           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
         }
    //  imshow("Contornos",drawing);
     // cv::waitKey(30);
      edges.release();
}

float CameraNode::perimeter(const std::vector<cv::Point2f> &a){
    float result=0, dx, dy;
    for (int i = 0; i < a.size(); i++){
                    dx = a[i].x - a[(i + 1) % a.size()].x;
                    dy = a[i].y - a[(i + 1) % a.size()].y;

                    result += std::sqrt(dx * dx + dy * dy);
    }
    return result;
}

void CameraNode::findCandidates(void){
    this->minContourLengthAllowed = 100.0;
    this->maxContourLengthAllowed = 4000.0;
    posiblesMarcas.clear();
    std::vector<cv::Point> aprox;
    for (unsigned int i = 0; i < contours.size(); i++){
         //Precisión en la aproximación de la curva
        double eps = contours.at(i).size() * .1;
        cv::approxPolyDP(contours.at(i), aprox, eps, true);
        if (aprox.size() == CURVE_SIZE && cv::isContourConvex(aprox)){
                float minDist = std::numeric_limits<float>::max();
                for (int j = 0; j < aprox.size(); j++){
                           cv::Point side = aprox.at(j) - aprox.at((j + 1) % 4);
                           minDist = std::min(minDist, (float)side.dot(side));
                }
                if (minDist > this->minContourLengthAllowed){
                            Marcador marker;
                            for (int j = 0; j < aprox.size(); j++){
                                       marker.addPoint(cv::Point2f(aprox.at(j).x, aprox.at(j).y));
                            }
                            cv::Point2f v1 = marker.getPoint(1) - marker.getPoint(0);
                            cv::Point2f v2 = marker.getPoint(2) - marker.getPoint(0);
                            double o = (v1.x * v2.y) - (v1.y * v2.x);
                            if (o < 0.0){
                                    cv::Point2f auxPoint = marker.getPoint(1);
                                    marker.setPoint(marker.getPoint(3), 1);
                                    marker.setPoint(auxPoint, 3);
                            }
                            marker.setContourIdx(i);
                            marker.setRotatedRect(cv::minAreaRect(aprox));
                            marker.setArea(cv::contourArea(aprox));
                            posiblesMarcas.push_back(marker);
                }


            }

        }
        std::vector<std::pair<int, int> > closestCandidates;

            for (int i = 0; i < posiblesMarcas.size(); i++){
                    Marcador markerA = posiblesMarcas.at(i);
                    for (int j = i + 1; j < posiblesMarcas.size(); j++){
                            Marcador markerB = posiblesMarcas.at(j);
                            float distSquared = 0;
                            for (int k = 0; k < CURVE_SIZE; k++){
                                    cv::Point v = markerA.getPoint(k) - markerB.getPoint(k);
                                    distSquared += v.dot(v);

                            }
                            distSquared /= 4;
                            if (distSquared < 100){
                                    closestCandidates.push_back(std::pair<int, int>(i, j));
                            }
                    }
    }

            std::vector<bool> removalMask(posiblesMarcas.size(), false);
                    for (int i = 0; i < closestCandidates.size(); i++){
                            float p1 = perimeter(posiblesMarcas.at(closestCandidates.at(i).first).getMarkerPoints());
                            float p2 = perimeter(posiblesMarcas.at(closestCandidates.at(i).second).getMarkerPoints());

                            int index;
                            if (p1 > p2){
                                    index = closestCandidates.at(i).first;
                            } else {
                                    index = closestCandidates.at(i).second;
                            }
                            removalMask.at(index) = true;
            }
                    Marcadores.clear();
                            for (int i = 0; i < posiblesMarcas.size(); i++){
                                    if (!removalMask[i])
                                            Marcadores.push_back(posiblesMarcas[i]);
                            }
                           //Dibujar candidatos
                            imshow("Detection",this->comb);
                            waitKey(30);
                            for (int i=0; i< Marcadores.size();i++){
                                    vector<Point2f> puntos = Marcadores[i].getAllPoints();
                                    cout<<"Número de puntos"<<puntos.size()<<endl;
                                    line (this->comb,puntos[0], puntos[1],Scalar(0,255,0),4);
                                    line (this->comb,puntos[1], puntos[2],Scalar(0,255,0),4);
                                    line (this->comb,puntos[2], puntos[3],Scalar(0,255,0),4);
                                    line (this->comb,puntos[3], puntos[0],Scalar(0,255,0),4);
                                }
                            //cout<<comb.size()<<endl;
                            imshow("Detection",this->comb);
                            waitKey(30);
                    }
cv::Mat CameraNode::rotate(cv::Mat input)
{
        cv::Mat out;
        input.copyTo(out);
        for (int i = 0; i < input.rows; i++){
                for (int j = 0; j < input.cols; j++){
                        out.at<uchar>(i, j) = input.at<uchar>(input.cols - j - 1, i);
                }
        }
        return out;
}
int CameraNode::MarkerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, Marcador &marker){
    int result = 0;
            cv::Mat grey = inputGrayscale;
            cv::threshold(grey, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            int cellHeigth = inputGrayscale.rows / CELL_MARKER_SIZE;
            int cellWidth = inputGrayscale.cols / CELL_MARKER_SIZE;
            for (int y = 0; y < CELL_MARKER_SIZE; y++){
                    int inc = 6;
                    if (y == 0 || y == 6){
                            inc = 1;
                    }

                    for (int x = 0; x < CELL_MARKER_SIZE; x += inc){
                            int cellX = x * cellWidth;
                            int cellY = y * cellHeigth;
                            cv::Mat cell = grey(cv::Rect(cellX, cellY, cellWidth, cellHeigth));
                            int nZ = cv::countNonZero(cell);
                            if (nZ >(cellWidth*cellHeigth) / 2){
                                    result = -1;
                            }
                    }
                    if (result == 0){
                                    cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);
                                    for (int y = 0; y < 5; y++){
                                            for (int x = 0; x < 5; x++){
                                                    int cellX = (x + 1) * cellWidth;
                                                    int cellY = (y + 1) * cellHeigth;
                                                    cv::Mat cell = grey(cv::Rect(cellX, cellY, cellWidth, cellHeigth));
                                                    int nZ = cv::countNonZero(cell);
                                                    if (nZ >(cellWidth*cellHeigth) / 2){
                                                            bitMatrix.at<uchar>(y, x) = 1;
                                                    }
                                            }
                                    }

                                    cv::Mat rotations[CURVE_SIZE];
                                    int distances[CURVE_SIZE];
                                    rotations[0] = bitMatrix;
                                    distances[0] = hammingDistance(bitMatrix);

                                    std::pair<int, int> minDist(distances[0], 0);

                                    for (int i = 1; i < CURVE_SIZE; i++){
                                            //get the hamming distance to the nearest possible word
                                            rotations[i] = rotate(rotations[i - 1]);
                                            distances[i] = hammingDistance(rotations[i]);

                                            if (distances[i] < minDist.first){
                                                    minDist.first = distances[i];
                                                    minDist.second = i;
                                            }
                                    }

                                    nRrotations = minDist.second;
                                    if (minDist.first == 0){
                                int mapId = 0, sectorId = 0, markerId = 0;
                                markerIdNumber(rotations[nRrotations], mapId, sectorId, markerId);
                                marker.setMapId(mapId);
                                marker.setSectorId(sectorId);
                                marker.setMarkerId(markerId);
                                std::string etiqueta ;
                                ostringstream convert;
                                convert<<markerId;
                                etiqueta=convert.str();
                                putText(this->comb, etiqueta, marker.getPoint(0),CV_FONT_HERSHEY_COMPLEX,0.8,Scalar(0,0,255));
                                //imshow("DetectionEti",this->comb);

                            } else {
                                result = -1;
                            }
                            }
                            return result;
                    }
    }

void CameraNode::recognizeMarkers(){
        std::vector<Marcador> goodMarkers;
        for (int i = 0; i < this->Marcadores.size(); i++){
                Marcador marker = this->Marcadores.at(i);
                Mat markerTransform=getPerspectiveTransform(marker.getMarkerPoints(), this->markerCorners);
                cv::warpPerspective(this->gray, canonicalMarkerImage, markerTransform, markerSize);
                int rotations = 0;
                if (MarkerDecoder(canonicalMarkerImage, rotations, marker) == 0){
                        std::vector<cv::Point2f> markerDots = marker.getMarkerPoints();
                        std::rotate(markerDots.begin(), markerDots.begin() + 4 - rotations, markerDots.end());
                        marker.MarkerPoints(markerDots);
                        goodMarkers.push_back(marker);
                }
        }
        OptMarkers = goodMarkers;
}

float CameraNode::linearInterpolator(const float& x, const cv::Point p1, const cv::Point p2){
        return (((x - p1.x)/(p2.x - p1.x) * (p2.x - p1.x)) + p1.x);
}
void CameraNode::poseEstimation(void){
    for (size_t i = 0; i < OptMarkers.size(); i++){
                    Marcador marker = OptMarkers.at(i);
                    cv::Point markerCenter = marker.getRotatedRect().center;
                    double angleInRadians =linearInterpolator((float)markerCenter.x, cv::Point(0, 2 * M_PI), cv::Point((float)IMAGE_WIDTH, 0));

                    if(angleInRadians > M_PI){
                            angleInRadians = angleInRadians - 2 * M_PI;
                    } else if(angleInRadians < -M_PI){
                            angleInRadians = angleInRadians + 2 * M_PI;
                    }
                    marker.setAngle(angleInRadians);
    }

}


int CameraNode::hammingDistance(cv::Mat bits){
    int ids[2][5] = {
                    { 1, 0, 0, 0, 1 },
                    { 1, 0, 1, 1, 1 }
            };

            int dist = 0;
            int sum = 0;

            //Compares the first and last row of the bit matrix with the template matrix ids

            for (int x = 0; x < 5; x++){
                    sum += bits.at<uchar>(0, x) == ids[0][x] ? 0 : 1;
            }

            if (1e5 > sum){
                    dist = sum;
            }

            sum = 0;

            for (int x = 0; x < 5; x++){
                    sum += bits.at<uchar>(4, x) == ids[1][x] ? 0 : 1;
            }

            dist += sum;

    return dist;
}

void CameraNode::markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId){

    for (int j = 0; j < 5; j++){
        if(bits.at<uchar>(1, j)){
                mapId += (16 / std::pow(2, j)) * bits.at<uchar>(1, j);
        }
    }
    for (int j = 0; j < 5; j++){
        if(bits.at<uchar>(2, j)){
                sectorId += (16 / std::pow(2, j)) * bits.at<uchar>(2, j);
        }
    }
    for (int j = 0; j < 5; j++){
        if(bits.at<uchar>(3, j)){
                markerId += (16 / std::pow(2,j)) * bits.at<uchar>(3, j);
        }
    }
}

int CameraNode::getMapSize(void){
    return this->map.size();
}






