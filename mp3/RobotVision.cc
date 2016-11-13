#include "RobotVision.hh"
#include <algorithm>
#include <dirent.h>

using namespace cv;
using namespace cv::xfeatures2d;

using std::chrono::duration;
using std::chrono::steady_clock;
using std::cout;
using std::endl;
using std::string;
using std::vector;

vector<QueryImage> RobotVision::query_images;
queue<Mat> RobotVision::image_queue;
queue<string> RobotVision::objects_found;
bool RobotVision::lamp_found = false;
int RobotVision::found_objects_count = 0;
pthread_mutex_t RobotVision::identify_mutex;

RobotVision::RobotVision() {
    this->directionVector = Point2f(0.f,1.f);
    this->waypoints = std::vector<Point>();
    this->currWaypoint = Point(0,0);
    this->prevWaypointTime = time(NULL);
    this->waypoints.push_back(this->currWaypoint);      
}   

void RobotVision::updateDirectionVector(float rotationAngle){
    cout << "Begin Update" << endl;
	float angleRadian = 3.14159265*rotationAngle/180;
    Point2f prevDirVector = this->directionVector;
    this->directionVector.x = prevDirVector.x*cos(angleRadian) - prevDirVector.y*sin(angleRadian);
    this->directionVector.y = prevDirVector.x*sin(angleRadian) + prevDirVector.y*cos(angleRadian);
    cout << "End Update" << endl;
}

void RobotVision::addNewWaypoint(int robotSpeed){
    cout << "End add waypoint" << endl;
    time_t now = time(NULL);
    time_t travelTime = now - this->prevWaypointTime;   
    float distance = robotSpeed * (float) travelTime;
	cout << distance << endl;
    this->currWaypoint.x += distance*this->directionVector.x;
	this->currWaypoint.y += distance*this->directionVector.y;
    this->waypoints.push_back(this->currWaypoint);
    this->prevWaypointTime = now;
    cout << "Begin add waypoint" << endl;
}

void RobotVision::drawContourMap(){
    cout << "Begin draw" << endl;
    // Create a drawing context, and use white background.
    Mat img_output(1200, 1600, CV_8UC3, Scalar(255, 255, 255));
    // // Plot the waypoints using blue color.
    Scalar lineColor(255, 0, 0);
    int lineWidth = 3;
    int radius = 5;
    auto waypoints = this->waypoints;
    
    // // Draw the bounding rectangle using orange color
    auto bound = boundingRect(waypoints);
	auto scale = min(1600.f/bound.width, 1200.f/bound.height); 	
    
	for(auto waypoint : waypoints){
        if(waypoint.y < 0)
            waypoint*=-1;
        waypoint.x *= scale;
        waypoint.y *= scale;
    }
    bound = boundingRect(waypoints);
    for (int i = 0; i < waypoints.size()-1; i++) {
        line(img_output, waypoints[i], waypoints[i + 1], lineColor, lineWidth, CV_AA);
         circle(img_output, waypoints[i], radius, lineColor, CV_FILLED, CV_AA    );  
     }
	rectangle(img_output, bound, Scalar(0, 1200, 1600));
    // // Finally store it as a png file
    imwrite("irobot_plot.png", img_output);
    cout << "End draw" << endl;
}
                

void * RobotVision::objectIdentification(void * args) {
  // Object identification setup
  DIR *pDIR;
  struct dirent * image;
  string dir = "./query-image/low-resolution";
  int num = 0;
  QueryImage lamp;
  if(pDIR = opendir(dir.c_str())) {
      while(image = readdir(pDIR)) {
          if(strcmp(image->d_name, ".") != 0 && strcmp(image->d_name, "..") != 0) {
              cout << dir + "/" + image->d_name << endl;
              Mat image_mat = imread(dir + "/" + image->d_name, IMREAD_GRAYSCALE);
              if(strcmp(image->d_name, "magic-lamp-600.jpg") == 0) {
                lamp = {
                  image->d_name,
                  image_mat
                };
              }
              QueryImage input = {
                image->d_name,
                image_mat
              };
              query_images.push_back(input);
          }
      }
  }
  try {
    raspicam::RaspiCam_Cv Camera;
    cv::Mat rgb_image, bgr_image;
    if(!Camera.open()) {
      cerr << "Error opening camera" << endl;
      return NULL;
    }
    cout << "Starting objectIdentifaction Thread" << endl;
    RobotSafetyStruct * info = (RobotSafetyStruct *) args;
    pthread_mutex_t * stream_mutex = info->stream_mutex;
    Create *robot = info->robot;
    int speed = info->speed;
    bool * turning = info->turning;
    bool * moving = info->moving;
    pthread_cond_t *cv = info->cv;

    pthread_mutex_lock(stream_mutex);
    bool local_moving = *moving;
    pthread_mutex_unlock(stream_mutex);
    
    while(local_moving) {
      pthread_mutex_lock(stream_mutex);
      while (*turning) {
        pthread_cond_wait(cv, stream_mutex);
      }

      robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
      this_thread::sleep_for(chrono::milliseconds(400));
      Camera.grab();
      Camera.retrieve (bgr_image);
      cout << "Retrieved Image" << endl;

      //@TODO: Make robot keep moving forward
      while (*turning) {
        pthread_cond_wait(cv, stream_mutex);
      }
      robot->sendDriveCommand(speed, Create::DRIVE_STRAIGHT);  
      pthread_mutex_unlock(stream_mutex);

      cout << "Unlocking" << endl;
      if(!lamp_found) {
        num ++;
        if(identify(lamp.image, bgr_image, "")) {
          //Stop the robot and disarm the lamp
          pthread_mutex_lock(stream_mutex);
          while (*turning) {
            pthread_cond_wait(cv, stream_mutex);
          }
          robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
          robot->sendLedCommand (Create::LED_ADVANCE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
          this_thread::sleep_for(chrono::milliseconds(2000));
          robot->sendLedCommand (Create::LED_ADVANCE, Create::LED_COLOR_RED, Create::LED_INTENSITY_OFF);
          pthread_mutex_unlock(stream_mutex);
          lamp_found = true;
        }
      }
      image_queue.push(bgr_image);
      this_thread::sleep_for(chrono::milliseconds(1000));
      pthread_mutex_lock(stream_mutex);
      local_moving = *moving;
      pthread_mutex_unlock(stream_mutex);
      cout << local_moving << endl;
    }
    robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);  //In case we pressed the play button before we set the speed
    //We are finished with moving. Run the object Identification here.
    identifyAndOutput();

  } catch (InvalidArgument& e) {
    cerr << e.what() << endl;
    return NULL;
  } catch (CommandNotAvailable& e) {
    cerr << e.what() << endl;
    return NULL;
  }
  return NULL;
    
}

void RobotVision::identifyAndOutput() {
  pthread_t workers[4];
  for(int i = 0; i < 4; i ++) {
    pthread_create(&workers[i], NULL, &RobotVision::runIdentify, NULL);
  }
  for(int i = 0; i < 4; i ++) {
    pthread_join(workers[i], NULL);
  }
  cout << "Done" << endl;
  ofstream myfile;
  myfile.open("./found_images/found_images.txt", ofstream::out | ofstream::app);
  myfile << "Found " << to_string(found_objects_count) << " images\n" << endl;
  while(!objects_found.empty()) {
    myfile << "Found: " << objects_found.front() << "\n\n";
    objects_found.pop();
  }
  myfile.close();
}

void * RobotVision::runIdentify(void * args) {
  pthread_mutex_lock(&identify_mutex);
  bool empty = image_queue.empty();
  while(!empty) {
    Mat scene_image = image_queue.front();
    image_queue.pop();
    int size = query_images.size();
    pthread_mutex_unlock(&identify_mutex);
    for(int i = 0; i < size; i ++) {
      pthread_mutex_lock(&identify_mutex);
      int local_found_count = found_objects_count;
      pthread_mutex_unlock(&identify_mutex);
      if(identify(query_images[i].image, scene_image, "./found_images/found_image_" + to_string(++local_found_count) + ".jpg")) {
          pthread_mutex_lock(&identify_mutex);
          found_objects_count ++;
          objects_found.push(query_images[i].name);
          query_images.erase(query_images.begin()+i);
          size --;
          pthread_mutex_unlock(&identify_mutex);
      }
    }
    pthread_mutex_lock(&identify_mutex);
    empty = image_queue.empty();
  }
  pthread_mutex_unlock(&identify_mutex);
  cout << "No more images" << endl;
  return NULL;
}


bool RobotVision::identify(Mat& img_query, Mat& scene_image_full, string output_file_name) {
    try {
        Mat img_scene;
        // Crop bottom
        // Images taken by mounting the camera on the robot will have some portion
        // of the side of the robot at the bottom. To reduce ambiguity during
        // detection and to speed up feature extraction, we crop it.
        // The fraction of cropping will be different depending on where the camera
        // is mounted on the robot. We find the useful portion of the picture is
        // the top 62.5% when camera mounted on front. When camera mounted on the
        // left side its the top 85% that contains useful information.
        cropBottom(scene_image_full, img_scene, 0.85);
        if (strcmp(output_file_name.c_str(), "") != 0) {
            imwrite("./out/scene.jpg", img_scene);
            imwrite("./out/full_scene.jpg", scene_image_full);
        }
        // Detect the keypoints and extract descriptors using SURF
        // Surf keypoint detector and descriptor.
        int minHessian = 100;
        int nOctaves = 4;
        int nOctaveLayers = 3;
        Ptr<SURF> detector = SURF::create(
            minHessian, nOctaves, nOctaveLayers, true);


        vector<KeyPoint> keypoints_query, keypoints_scene;
        Mat descriptors_query, descriptors_scene;

        detector->detectAndCompute(
            img_scene, Mat(), keypoints_scene, descriptors_scene);
        detector->detectAndCompute(
            img_query, Mat(), keypoints_query, descriptors_query);

        // Matching descriptor vectors using Brute Force matcher
        BFMatcher matcher(NORM_L2);
        vector<vector<DMatch>> matches;
        matcher.knnMatch(descriptors_query, descriptors_scene, matches, 2);

        vector<DMatch> good_matches;
        for(int i = 0; i < descriptors_query.rows; i++) {
          if (matches[i][0].distance < 0.75 * matches[i][1].distance)
            good_matches.push_back(matches[i][0]);
        }

        // Find the location of the query in the scene
        vector<Point2f> query;
        vector<Point2f> scene;
        for(size_t i = 0; i < good_matches.size(); i++) {
          query.push_back(keypoints_query[good_matches[i].queryIdx].pt);
          scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
        }

        vector<Point2f> scene_corners(4);
        bool res = alignPerspective(
            query, scene, img_query, img_scene, scene_corners);
        cout << "Matching and alignment" << endl;

        if (res) {  
          cout << "Object found" << endl;
          //We don't need to save anything if its the magic lamp
          if (strcmp(output_file_name.c_str(), "") != 0) {
            // Write output to file
            Mat img_matches;
            drawMatches(img_query, keypoints_query, img_scene, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            // Fill the extra area in almost white (Saves ink when printing)
            if (img_query.rows < img_scene.rows) {
              rectangle(img_matches, Point2f(0, img_query.rows),
                  Point2f(img_query.cols - 1, img_scene.rows - 1),
                  Scalar(255, 240, 240), CV_FILLED);
            } else if (img_scene.rows < img_query.rows) {
              rectangle(img_matches, Point2f(img_query.cols, img_scene.rows),
                  Point2f(img_query.cols + img_scene.cols - 1, img_query.rows - 1),
                  Scalar(255, 240, 240), CV_FILLED);
            }
            drawProjection(img_matches, img_query, scene_corners);
            // Write result to a file
            cv::imwrite(output_file_name, img_matches);
          }
        } else {
          cout << "Object not found" << endl;
        }
        return res;
    } catch (cv::Exception& e) {
        cout << "exception caught: " << e.what();
        return false;
    }
    return false;
}


void RobotVision::cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction) {
  // Crop the lower part of the scene
  cv::Rect crop;
  crop.x = 0;
  crop.y = 0;
  crop.width = img_scene_full.size().width;
  crop.height = img_scene_full.size().height * crop_fraction;
  img_scene = img_scene_full(crop);
}


bool RobotVision::alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
    Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners) {
 // cout << "Scene find Homography" << scene << endl;
Mat H = findHomography(query, scene, RANSAC);
  if (H.rows == 0 && H.cols == 0) {
    cout << "Failed rule0: Empty homography" << endl;
    return false;
  }

  vector<Point2f> query_corners(4);
  query_corners[0] = cvPoint(0,0);
  query_corners[1] = cvPoint(img_query.cols, 0);
  query_corners[2] = cvPoint(img_query.cols, img_query.rows);
  query_corners[3] = cvPoint(0, img_query.rows );

  perspectiveTransform(query_corners, scene_corners, H);

  float min_area = 32.0 * 32.0;
  double max_area = img_scene.rows * img_scene.cols;
  float ratio_inside = 0.75;
  float min_angle_sin =  0.173; // Minimum 10 degree angle required

  // Geometric verification heuristics
  // Rule 1: Must be a convex hull.
  // Rule 2: Area can’t be less than 32x32
  // Rule 3: The detected projection can’t have more than 100% area
  // Rule 4: Projection can't contain very small angle < 10 degree
  // Rule 5: More than 75% of the area of the detected projection should have
  // to be within image bounds

  // Rule 1: Must be a convex hull.
  vector<Point2f> sc_vec(4);
  // Generate 4 vectors from the 4 scene corners
  for(int i = 0; i < 4; i++) {
    sc_vec[i] = scene_corners[(i + 1) % 4] - scene_corners[i];
  }
  vector<float> sc_cross(4);
  // Calculate cross product of pairwise vectors
  for(int i = 0; i < 4; i++) {
    sc_cross[i] = sc_vec[i].cross(sc_vec[(i+1) % 4]);
  }

  // Check for convex hull
  if (!(sc_cross[0] < 0 && sc_cross[1] < 0 && sc_cross[2] < 0 && sc_cross[3] < 0)
      && !(sc_cross[0] > 0 && sc_cross[1] > 0 && sc_cross[2] > 0 && sc_cross[3] > 0)) {
    cout << "Failed rule1: Not a convex hull" << endl;
    return false;
  }

  // Rule 2: Area can’t be less than 32x32
  // Rule 3: The detected projection can’t have more than 100% area
  float area = (sc_cross[0] + sc_cross[2]) / 2.0;
  if (fabs(area) < min_area) {
    cout << "Failed rule2: Projection too small" << endl;
    return false;
  } else if (fabs(area) > max_area) {
    cout << "Failed rule3: Projection too large" << endl;
    return false;
  }

  // Rule 4: Can't contain very small angle < 10 degree inside projection.
  // Check for angles
  vector<float> sc_norm(4);
  for (int i = 0; i < 4; i++) {
    sc_norm[i] = norm(sc_vec[i]);
  }
  for (int i = 0; i < 4; i++) {
    float sint = sc_cross[i] / (sc_norm[i] * sc_norm[(i + 1) % 4]);
    if (fabs(sint) < min_angle_sin) {
      cout << "Failed rule4: Contains very small angle" << endl;
      return false;
    }
  }

  // Rule 5: More than 75% of the area of the detected projection should
  // have to be within image bounds.
  // Approximate mechanism by determining the bounding rectangle.
  cv::Rect bound = boundingRect(scene_corners);
  cv::Rect scene_rect(0.0, 0.0, img_scene.cols, img_scene.rows);
  cv::Rect isect = bound & scene_rect;
  if (isect.width * isect.height <  ratio_inside * bound.width * bound.height ) {
    cout << "Failed rule5: Large proportion outside scene" << endl;
    return false;
  }
  return true;
}

// Show the projection
void RobotVision::drawProjection(Mat& img_matches, Mat& img_query,
    vector<Point2f>& scene_corners) {
  line(img_matches, scene_corners[0] + Point2f(img_query.cols, 0),
      scene_corners[1] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[1] + Point2f(img_query.cols, 0),
      scene_corners[2] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[2] + Point2f(img_query.cols, 0),
      scene_corners[3] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[3] + Point2f(img_query.cols, 0),
      scene_corners[0] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
}

string RobotVision::type2str(int type) {
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans + '0');
  return r;
}
