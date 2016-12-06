#include <SerialStream.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <mutex>
#include <vector>
#include <signal.h>
#include <string.h>
#include <raspicam/raspicam_cv.h>
#include "RobotSensors.hh"
#include "irobot-create.hh"
#include "RobotSafety.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

pthread_mutex_t stream_mutex;
int speed = 140; // 50
int current_wall_signal = 0;
int prev_wall_signal = 0;

RobotSensors * sensorCache;

/* Pure gain */
double controller(double error) {
  return error * 0.08;
}

void robotModel(Create * robot, double theta) {
  pthread_mutex_lock(&stream_mutex);
  robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  if (theta > 0) {
      robot->sendDriveCommand(70, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  } else {
      robot->sendDriveCommand(70, Create::DRIVE_INPLACE_CLOCKWISE);
  }
  robot->sendWaitAngleCommand(theta);
  robot->sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
  pthread_mutex_unlock(&stream_mutex);
}

double sensorModel() {
  short wallSignal = sensorCache->getWallSignal();
  double distance = (180 - wallSignal) / 2.62;
  cout << "distance: " << distance << endl;
  return distance;
}

void * mainThread(void * args) {
  RobotSafetyStruct * info = (RobotSafetyStruct *) args;

  Create * robot = info->robot;

  this_thread::sleep_for(chrono::milliseconds(2000));

  pthread_mutex_lock(&stream_mutex);
  robot->sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
  pthread_mutex_unlock(&stream_mutex);
  double desiredDistance = 250; /* millimeters */
  while (1) {
    double actualDistance = sensorModel();
    double error = desiredDistance - actualDistance;
    double theta = controller(error);
    robotModel(robot, theta);
    this_thread::sleep_for(chrono::milliseconds(360));
  }
  cout << "Done Moving" << endl;
  return NULL;
}

int main(int argc, char** argv)
{
  char serial_loc[] = "/dev/ttyUSB0";

  try {
    SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
    cout << "Opened Serial Stream to" << serial_loc << endl;
    this_thread::sleep_for(chrono::milliseconds(1000));
    Create robot(stream);
    cout << "Created iRobot Object" << endl;
    robot.sendFullCommand();
    cout << "Setting iRobot to Full Mode" << endl;
    this_thread::sleep_for(chrono::milliseconds(1000));
    cout << "Robot is ready" << endl;
  
    // Set up the song to play
    Create::note_t note1(50, 50);
    Create::song_t song;
    song.push_back(note1);  
    robot.sendSongCommand(0, song);

    // Let's stream some sensors.
    Create::sensorPackets_t sensors;
    sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS); /* 1 data byte */
    sensors.push_back(Create::SENSOR_OVERCURRENTS); /* 1 data byte + 2 possible unused bytes */
    sensors.push_back(Create::SENSOR_WALL_SIGNAL); /* 2 data bytes */
    sensors.push_back(Create::SENSOR_CLIFF_LEFT_SIGNAL);
    sensors.push_back(Create::SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
    sensors.push_back(Create::SENSOR_CLIFF_FRONT_RIGHT_SIGNAL);
    sensors.push_back(Create::SENSOR_CLIFF_RIGHT_SIGNAL);
    sensors.push_back(Create::SENSOR_BUTTONS);
    sensors.push_back(Create::SENSOR_ANGLE);
    robot.sendStreamCommand(sensors);
    cout << "Sent Stream Command" << endl;
      
    pthread_mutex_init(&stream_mutex, NULL);

    sensorCache = new RobotSensors(robot, &stream_mutex);

    //Start the Robot safety threads
    pthread_t main_thread, overcurrent_thread, cliff_thread, sensor_thread;
    pthread_attr_t mainAttr, OCAttr, cliffAttr, sensorAttr;
    struct sched_param mainParam, OCParam, cliffParam, sensorParam;
 
    pthread_attr_init(&mainAttr);
    pthread_attr_init(&OCAttr);
    pthread_attr_init(&cliffAttr);
    pthread_attr_init(&sensorAttr);
  
    pthread_attr_setinheritsched(&mainAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setinheritsched(&OCAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setinheritsched(&cliffAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setinheritsched(&sensorAttr, PTHREAD_EXPLICIT_SCHED);

    pthread_attr_getschedparam(&mainAttr, &mainParam);
    pthread_attr_getschedparam(&OCAttr, &OCParam);
    pthread_attr_getschedparam(&cliffAttr, &cliffParam);
    pthread_attr_getschedparam(&sensorAttr, &sensorParam);
      
    mainParam.sched_priority = 95;
    OCParam.sched_priority = 18;
    cliffParam.sched_priority = 20;
    sensorParam.sched_priority = 90;

    pthread_attr_setschedpolicy(&mainAttr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&OCAttr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&cliffAttr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&sensorAttr, SCHED_FIFO);

    pthread_attr_setschedparam(&mainAttr, &mainParam);
    pthread_attr_setschedparam(&OCAttr, &OCParam);
    pthread_attr_setschedparam(&cliffAttr, &cliffParam);
    pthread_attr_setschedparam(&sensorAttr, &sensorParam);

    RobotSafetyStruct thread_info;
    thread_info.speed = speed;
    thread_info.robot = &robot;
    thread_info.stream_mutex = &stream_mutex;
    thread_info.sensorCache = sensorCache;

    pthread_create(&main_thread, &mainAttr, &mainThread, &thread_info);
    pthread_create(&overcurrent_thread, &OCAttr, &RobotSafety::overcurrent, &thread_info);
    pthread_create(&cliff_thread, &cliffAttr, &RobotSafety::cliffWheelDrop, &thread_info);
    pthread_create(&sensor_thread, &sensorAttr, &RobotSensors::startUpdateValues, sensorCache);
    
    cout << "After Create" << endl;
    sleep(10);

    pthread_join(main_thread, NULL);
    pthread_join(overcurrent_thread, NULL);
    pthread_join(cliff_thread, NULL);
    pthread_join(sensor_thread, NULL);

    pthread_attr_destroy(&mainAttr);
    pthread_attr_destroy(&OCAttr);
    pthread_attr_destroy(&cliffAttr);
    pthread_attr_destroy(&sensorAttr);
  }   
  catch (InvalidArgument& e)
  {
    cerr << e.what () << endl;
    return 3;
  }
  catch (CommandNotAvailable& e)
  {
    cerr << e.what () << endl;
    return 4;
  }
}
