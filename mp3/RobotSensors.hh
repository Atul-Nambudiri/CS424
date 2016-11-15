#ifndef ROBOTVISION_HH
#define ROBOTVISION_HH

#include <pthread.h>

#include "irobot-create.hh"

using namespace iRobot;

class RobotSensors {

private:

  Create robot;
  pthread_mutex_t * stream_mutex;

  void updateValues();

  bool leftWheelOvercurrent;
  bool rightWheelOvercurrent;

  short cliffLeftSignal;
  short cliffRightSignal;
  short cliffFrontLeftSignal;
  short cliffFrontRightSignal;

  bool wheeldropLeft;
  bool wheeldropRight;
  bool wheeldropCaster;

  short angle;
  short wallSignal;
  bool bumpLeft;
  bool bumpRight;

public:

  RobotSensors(Create r, pthread_mutex_t * s_m) : robot(r), stream_mutex(s_m) {}

  static void * startUpdateValues(void * args);

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
  short getWallSignal();
  bool getBumpLeft();
  bool getBumpRight();

};

#endif
