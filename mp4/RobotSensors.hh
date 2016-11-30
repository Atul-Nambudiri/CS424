#ifndef ROBOTSENSORS_HH
#define ROBOTSENSORS_HH

#include <pthread.h>

#include "irobot-create.hh"

using namespace iRobot;

class RobotSensors {

public:

  RobotSensors(Create r, pthread_mutex_t * s_m) : robot(r), stream_mutex(s_m) {}

  bool getLeftWheelOvercurrent();
  bool getrightWheelOvercurrent();

  short getCliffLeftSignal();
  short getCliffRightSignal();
  short getCliffFrontLeftSignal();
  short getCliffFrontRightSignal();
  
  bool getWheeldropLeft();
  bool getWheeldropRight();
  bool getWheeldropCaster();

  short getAngle();
  short getPrevWallSignal();
  short getWallSignal();
  short getPlayButton();

  bool getBumpLeft();
  bool getBumpRight();

  static void * startUpdateValues(void * args);
  void updateValues();
  void beginCalculatingAngle();

private:

  Create robot;
  pthread_mutex_t * stream_mutex;

  volatile bool leftWheelOvercurrent;
  volatile bool rightWheelOvercurrent;

  volatile short cliffLeftSignal;
  volatile short cliffRightSignal;
  volatile short cliffFrontLeftSignal;
  volatile short cliffFrontRightSignal;

  volatile bool wheeldropLeft;
  volatile bool wheeldropRight;
  volatile bool wheeldropCaster;

  volatile short wallSignal;
  volatile short prevWallSignal;
  volatile short playButton;
  volatile bool bumpLeft;
  volatile bool bumpRight;
    
};

#endif
