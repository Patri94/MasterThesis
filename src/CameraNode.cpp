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
#include <trabajo/CameraNode.hpp>

#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75
#define CELL_MARKER_SIZE 7
#define CURVE_SIZE 4
#define IMAGE_WIDTH 320


using namespace cv;
using namespace std;

void CameraNode::Init(void){

    markerSize = cv::Size(215, 345);

    this->markerCorners.push_back(cv::Point2f(0, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, 0));
    this->markerCorners.push_back(cv::Point2f(markerSize.width - 1, markerSize.height - 1));
    this->markerCorners.push_back(cv::Point2f(0, markerSize.height - 1));


    this->MarkersTemplate.push_back(imread("Tag0.png"));
    this->MarkersTemplate.push_back(imread("Tag1.png"));
    this->MarkersTemplate.push_back(imread("Tag2.png"));
    this->MarkersTemplate.push_back(imread("Tag3.png"));
    this->MarkersTemplate.push_back(imread("Tag4.png"));
    this->MarkersTemplate.push_back(imread("Tag5.png"));


}

void CameraNode::Undistort(void){
    Mat camMatrix = cv::Mat(3, 3, CV_32F);
    camMatrix.at<float>(0, 0) = 8.5101024687735935e+02;
    camMatrix.at<float>(0, 1) = -2.2255059056366439e-01;
    camMatrix.at<float>(0, 2) = 6.5571465382877625e+02;
    camMatrix.at<float>(1, 0) = 0.0;
    camMatrix.at<float>(1, 1) = 8.5170243585411265e+02;;
    camMatrix.at<float>(1, 2) = 5.1216084358475405e+02;
    camMatrix.at<float>(2, 0) = 0.0;
    camMatrix.at<float>(2, 1) = 0.0;
    camMatrix.at<float>(2, 2) = 1.0;

    Mat distCoeff = cv::Mat(5, 1, CV_32F);
    distCoeff.at<float>(0, 0) = -2.6019586095779829e-001;
    distCoeff.at<float>(1, 0) = 5.5052401922323718e-002;
    distCoeff.at<float>(2, 0) = 0.0;
    distCoeff.at<float>(3, 0) = 0.0;
    distCoeff.at<float>(4, 0) = -4.5449850126361765e-003;
    if (!(this->comb.empty())){
         undistort(this->comb,this->ucomb,camMatrix,distCoeff);
         imshow("Undistorted",this->ucomb);
         waitKey();

     }

}

void CameraNode::imageCallback1(const sensor_msgs::ImageConstPtr& msg){
    this->cam1 = cv_bridge::toCvShare(msg, "bgr8")->image;
     //cv::imshow("view1", this->cam1);
    // cv::waitKey(30);
    //this->cam1=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam1).toImageMsg();
}

void CameraNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg){
    this->cam2 = cv_bridge::toCvShare(msg, "bgr8")->image;
     //cv::imshow("view2", this->cam2);
     //cv::waitKey(30);
    //this->cam1=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    this->ms2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam1).toImageMsg();
}

void CameraNode::imageCallback3(const sensor_msgs::ImageConstPtr& msg){
    this->cam3 = cv_bridge::toCvShare(msg, "bgr8")->image;
     //cv::imshow("view3", this->cam3);
     //cv::waitKey(30);
    //this->cam1=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    this->ms1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cam1).toImageMsg();
}
void CameraNode::Concatenar(void){
    //cout<<cam1.size()<<"****"<<endl;
    if (!(this->cam1.empty()) && !(this->cam2.empty()) && !(this->cam3.empty())){
    cv::hconcat(this->cam1,this->cam3,this->comb);
    cv::hconcat(this->comb,this->cam2,this->comb);
    cv::imshow("combi", this->comb);
    cv::waitKey(30);
        }
}
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

    cout<<this->brightnessAvg<<endl;
        }
}

void CameraNode::threshold(void){
    if(!(this->comb.empty())){
    cv::cvtColor(this->comb,this->gray, CV_BGR2GRAY);
    float index= 75.0 / 255.0 * this->brightnessAvg ;
    cv::adaptiveThreshold(this->gray,this->thresholded, 255,  CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD, (int)index);
    cv::imshow("thresh",this->thresholded);
    cv::waitKey(30);
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
    cout<<"He hecho canny"<<endl;
    cout<<edges.size()<<endl;
    waitKey();
    cv::findContours (edges,Todos,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cout<<"He hecho contornos"<<endl;
    cout<<Todos.size()<<endl;
    cv::waitKey();
    //FiltrarContornos
    for (unsigned int i = 0; i < Todos.size(); i++){
                    if (Todos.at(i).size() > minContourPointsAllowed){

                            contours.push_back(Todos.at(i));
                            cout<<i<<endl;
                    }
    }
    cout<<contours.size()<<endl;
    cv::waitKey();
    /// Draw contours
      Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
      for( int i = 0; i< contours.size(); i++ )
         {
           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         }
      cout<<drawing.size()<<endl;
      cv::waitKey();
      imshow("Contornos",drawing);
      cv::waitKey(30);
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
                                    //std::swap(marker.getPoint(1), marker.getPoint(3));
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
                            cout<<"Candidatos"<<endl;
                            cout<<Marcadores.size()<<endl;
                           //Dibujar candidatos
                            imshow("Detection",this->comb);
                            waitKey();
                            for (int i=0; i< Marcadores.size();i++){
                                    vector<Point2f> puntos = Marcadores[i].getAllPoints();
                                    cout<<"Número de puntos"<<puntos.size()<<endl;
                                    line (this->comb,puntos[0], puntos[1],Scalar(0,255,0),4);
                                    line (this->comb,puntos[1], puntos[2],Scalar(0,255,0),4);
                                    line (this->comb,puntos[2], puntos[3],Scalar(0,255,0),4);
                                    line (this->comb,puntos[3], puntos[0],Scalar(0,255,0),4);
                                }
                            cout<<comb.size()<<endl;
                            imshow("Detection",this->comb);
                            waitKey();
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
                cout<<"PerspectiveTransform esquinas en imagen"<<marker.getMarkerPoints().size()<<endl;
                cout<<"PerspectiveTransform esquinas en plantilla"<<markerCorners.size()<<endl;
                waitKey();
                Mat markerTransform=getPerspectiveTransform(marker.getMarkerPoints(), this->markerCorners);
                cv::warpPerspective(this->gray, canonicalMarkerImage, markerTransform, markerSize);
                //std::ostringstream windowName;

                int rotations = 0;
                if (MarkerDecoder(canonicalMarkerImage, rotations, marker) == 0){
                        std::vector<cv::Point2f> markerDots = marker.getMarkerPoints();
                        std::rotate(markerDots.begin(), markerDots.begin() + 4 - rotations, markerDots.end());
                        marker.MarkerPoints(markerDots);
                        goodMarkers.push_back(marker);
                }
        }
        cout<<"Cuantos"<<OptMarkers.size()<<endl;
        OptMarkers = goodMarkers;
        for (int i=0;i<goodMarkers.size();i++){
                cout<< "Marcador"<<i<<endl;
                cout<<"MapId"<<goodMarkers[i].getMapID()<<endl;
                cout<<"SectorId"<<goodMarkers[i].getSectorID()<<endl;
                cout<<"MarkerId"<<goodMarkers[i].getMarkerID()<<endl;
            }
        waitKey();
}

float CameraNode::linearInterpolator(const float& x, const cv::Point p1, const cv::Point p2){
        return (((x - p1.x)/(p2.x - p1.x) * (p2.x - p1.x)) + p1.x);
}
void CameraNode::poseEstimation(void){
    for (size_t i = 0; i < OptMarkers.size(); i++){
                    Marcador marker = OptMarkers.at(i);
                    cv::Point markerCenter = marker.getRotatedRect().center;
                    double angleInRadians =linearInterpolator((float)markerCenter.x, Point(0, 2 * M_PI), Point((float)IMAGE_WIDTH, 0));

                    if(angleInRadians > M_PI){
                            angleInRadians = angleInRadians - 2 * M_PI;
                    } else if(angleInRadians < -M_PI){
                            angleInRadians = angleInRadians + 2 * M_PI;
                    }
                    marker.setAngle(angleInRadians);
                    //RNUtils::printLn("Marker (%d) angle: %lf", i, angleInRadians);
    }

}



int CameraNode::hammingDistance(cv::Mat bits){
        int ids[4][5] = {
                { 1, 0, 0, 0, 1 },
                { 1, 0, 1, 1, 1 }
        };

        int dist = 0;

        for (int y = 0; y < 5; ){
                int minSum = 1e5; //hamming distance to each possible word
                for (int p = 0; p < 2; p++){
                        int sum = 0;
                        //now, count
                        for (int x = 0; x < 5; x++){
                                sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
                        }

                        if (minSum > sum){
                                minSum = sum;
                        }
                }
                y += 4;
                //do the and
                dist += minSum;
        }

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



