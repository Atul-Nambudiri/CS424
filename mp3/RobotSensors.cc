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

    wallSignal = robot.wallSignal();
    bumpLeft = robot.bumpLeft();
    bumpRight = robot.bumpRight();   
    playButton = robot.playButton();

    pthread_mutex_unlock(stream_mutex);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

short RobotSensors::getAngle() { // Doesn't work with mutex locks
  pthread_mutex_lock(stream_mutex);
  short angle = robot.angle();
  pthread_mutex_unlock(stream_mutex);
  return angle;
}
short RobotSensors::getWallSignal() {
  return wallSignal;
}
bool RobotSensors::getBumpLeft() {
  return bumpLeft;
}
bool RobotSensors::getBumpRight() {
  return bumpRight;
}
short RobotSensors::getPlayButton(){
    return playButton;
}
void RobotSensors::beginCalculatingAngle(){
    pthread_mutex_lock(stream_mutex);
    robot.angle();
    pthread_mutex_unlock(stream_mutex);
}
