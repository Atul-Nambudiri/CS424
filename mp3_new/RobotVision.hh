#ifndef ROBOTVISION_HH
#define ROBOTVISION_HH
#include "irobot-create.hh"
#include "RobotSafety.hh"
#include <SerialStream.h>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
#include <math.h>
#include <ctime>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>
#include <fstream>

#include <dirent.h>

using namespace iRobot;
using namespace LibSerial;
using namespace cv;
using namespace std;

struct QueryImage {
    string name;
    Mat image;
};

class RobotVision{
    
private: 
    Point2f directionVector;
    std::vector<Point> waypoints;
    Point currWaypoint;
    clock_t prevWaypointTime;
    clock_t rotationTime;
    static bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
        Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners);
    static bool identify(Mat& query_image, Mat& scene_image, string output_file_name);
    static void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction);
    static void drawProjection(Mat& img_matches, Mat& img_query,
        vector<Point2f>& scene_corners);
    static string type2str(int type);
public: 
    RobotVision();
    void updateDirectionVector(int rotationSpeed = 70, int radius = 170, double angle = 0.0); 
    void addNewWaypoint(int robotSpeed);
    void drawContourMap();
    void startRotating();
    static void identifyAndOutput();
    static void * runIdentify(void * args);
    static vector<QueryImage> query_images;
    static queue<Mat> image_queue;
    static queue<string> objects_found;
    static bool lamp_found;
    static int found_objects_count;
    static void * objectIdentification(void * args);
    static pthread_mutex_t identify_mutex;
   
};

#endif 
