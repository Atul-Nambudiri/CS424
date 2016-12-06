#include "RobotSensors.hh"
#include <chrono>
#include <thread>

void * RobotSensors::startUpdateValues(void * args) {
  RobotSensors * sensors = (RobotSensors*) args;
  sensors->updateValues();
  //does not return
  return NULL;
}

void RobotSensors::updateValues() {
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

    prevWallSignal = wallSignal;
    wallSignal = robot.wallSignal();
    angle = robot.angle();

    playButton = robot.playButton();

    bumpLeft = robot.bumpLeft();
    bumpRight = robot.bumpRight();   

    pthread_mutex_unlock(stream_mutex);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
}

bool RobotSensors::getLeftWheelOvercurrent() {
  return leftWheelOvercurrent;
}
bool RobotSensors::getrightWheelOvercurrent() {
  return rightWheelOvercurrent;
}

short RobotSensors::getCliffLeftSignal() {
  return cliffLeftSignal;
}
short RobotSensors::getCliffRightSignal() {
  return cliffRightSignal;
}
short RobotSensors::getCliffFrontLeftSignal() {
  return cliffFrontLeftSignal;
}
short RobotSensors::getCliffFrontRightSignal() {
  return cliffFrontRightSignal;
}

bool RobotSensors::getWheeldropLeft() {
  return wheeldropLeft;
}
bool RobotSensors::getWheeldropRight() {
  return wheeldropRight;
}
bool RobotSensors::getWheeldropCaster() {
  return wheeldropCaster;
}

short RobotSensors::getPrevWallSignal() {
    return prevWallSignal;
}
short RobotSensors::getWallSignal() {
  return wallSignal;
}
short RobotSensors::getAngle() {
  return angle;
}

bool RobotSensors::getPlayButton(){
    return playButton;
}

bool RobotSensors::getBumpLeft() {
  return bumpLeft;
}
bool RobotSensors::getBumpRight() {
  return bumpRight;
}
