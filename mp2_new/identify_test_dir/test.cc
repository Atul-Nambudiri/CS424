#include "RobotIdentification.hh"
#include <opencv2/imgproc.hpp>
#include <raspicam/raspicam_cv.h>
#include <iostream>

#include <dirent.h>

using namespace cv;
using namespace std;

int main() {
    RobotIdentification test;
    raspicam::RaspiCam_Cv Camera;
    //cv::Mat scene;
    if(!Camera.open()) {
      cerr << "Error opening camera" << endl;
      return 0;
    }
    Camera.grab();
    //Camera.retrieve (scene);
    Mat scene = imread("found3.jpg", IMREAD_GRAYSCALE);
    cout << "Read" << endl;
    test.runIdentify(scene);
}
