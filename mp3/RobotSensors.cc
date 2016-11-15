#include "RobotSensors.hh"

void * startUpdateValues(void * args) {
  RobotSensors * sensors = (RobotSensors*) args;
  sensors->updateValues();
  //does not return
  return NULL;
}

void updateValues() {
  while (1) {
    pthread_mutex_lock(stream_mutex);

    leftWheelOvercurrent = robot.leftWheelOvercurrent();
    rightWheelOvercurrent = robot.rightWheelOvercurrent();

    cliffLeftSignal = robot.cliffLeftSignal();
    cliffRightSignal = robot.cliffRightSignal();
    cliffFrontLeftSignal = robot.cliffFrontLeftSignal();
    cliffFrontRightSignal = robot.cliffFrontRightSignal();

    wheeldropLeft = robot.wheeldropLeft();
    wheeldropRight = robot.wheeldropRight();
    wheeldropCaster = robot.wheeldropCaster();

    angle = robot.angle();
    wallSignal = robot.wallSignal();
    bumpLeft = robot.bumpLeft();
    bumpRight = robot.bumpRight();

    pthread_mutex_unlock(stream_mutex);
    //sleep?
  }
}

bool getLeftWheelOvercurrent() {
  return leftWheelOvercurrent;
}
bool getrightWheelOvercurrent() {
  return rightWheelOvercurrent;
}

short getCliffLeftSignal() {
  return cliffLeftSignal;
}
short getCliffRightSignal() {
  return cliffRightSignal;
}
short getCliffFrontLeftSignal() {
  return cliffFrontLeftSignal;
}
short getCliffFrontRightSignal() {
  return cliffFrontRightSignal;
}

bool getWheeldropLeft() {
  return wheeldropLeft;
}
bool getWheeldropRight() {
  return wheeldropRight;
}
bool getWheeldropCaster() {
  return wheeldropCaster;
}

short getAngle() {
  return angle;
}
short getWallSignal() {
  return wallSignal;
}
bool getBumpLeft() {
  return bumpLeft;
}
bool getBumpRight() {
  return bumpRight;
}
