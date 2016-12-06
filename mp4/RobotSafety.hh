#ifndef ROBOTSAFETY_HH
#define ROBOTSAFETY_HH
#include "irobot-create.hh"
#include "RobotSensors.hh"
#include <mutex>
#include <unistd.h>
#include <pthread.h>
#include <thread>

using namespace iRobot;
using namespace LibSerial;
using namespace std;

typedef struct RobotSafetyStruct {
	int speed;
	Create * robot;
	pthread_mutex_t * stream_mutex;
  RobotSensors * sensorCache;
} RobotSafetyStruct;

class RobotSafety {

  RobotSafety() {}

public:

  static void stopAndPlaySong(pthread_mutex_t * stream_mutex, Create * robot);
  static void startAgain(pthread_mutex_t * stream_mutex, Create * robot, int speed);
  static void * overcurrent(void * args);
  static void * cliffWheelDrop(void * args);
};

#endif
