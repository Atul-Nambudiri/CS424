#include "RobotSafety.hh"

void RobotSafety::stopAndPlaySong(pthread_mutex_t * stream_mutex, Create * robot) {
  //stop robot and play song
  pthread_mutex_lock(stream_mutex);
  robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);			
  robot->sendPlaySongCommand(0);
  sleep(1);
  robot->sendPlaySongCommand(0);
  pthread_mutex_unlock(stream_mutex);
}

void RobotSafety::startAgain(pthread_mutex_t * stream_mutex, Create * robot, int speed) {
  pthread_mutex_lock(stream_mutex);
  robot->sendDriveCommand(speed, Create::DRIVE_STRAIGHT);  
  pthread_mutex_unlock(stream_mutex);
}

void * RobotSafety::overcurrent(void * args) {
  RobotSafetyStruct * info = (RobotSafetyStruct *) args;

  int speed = info->speed;
  Create * robot = info->robot;
  pthread_mutex_t * stream_mutex = info->stream_mutex;
  RobotSensors * sensors = info->sensorCache;

  this_thread::sleep_for(chrono::milliseconds(1000)); // So it doens't beep in the beginning

  cout << "Overcurrent Thread Starting" << endl;
  while (true) {
    //Check for overcurrent in either wheel
    bool left = sensors->getLeftWheelOvercurrent();
    bool right = sensors->getrightWheelOvercurrent();


    if (left && right) {
      cout << "overcurrent detected" << endl;
      this_thread::sleep_for(chrono::milliseconds(1000));

      //check again
      bool left = sensors->getLeftWheelOvercurrent();
      bool right = sensors->getrightWheelOvercurrent();

      if (left && right) {
        cout << "overcurrent confirmed" << endl;

        //stop robot and play song
        stopAndPlaySong(stream_mutex, robot);
        while (!sensors->getPlayButton()) {/* do nothing */}
        startAgain(stream_mutex, robot, speed);
      }
    }
  }

  return NULL;
}

void * RobotSafety::cliffWheelDrop(void * args) {
  RobotSafetyStruct * info = (RobotSafetyStruct *) args;

  int speed = info->speed;
  Create * robot = info->robot;
  pthread_mutex_t * stream_mutex = info->stream_mutex;
  RobotSensors * sensors = info->sensorCache;
	
  this_thread::sleep_for(chrono::milliseconds(1000)); // So it doens't beep in the beginning

  int cliffCounter = 0;

  bool error = false;
  cout << "Cliff Thread Starting" << endl;
  cout << "Wheel Drop Thread Starting" << endl;
  while (true) {
    short left = sensors->getCliffLeftSignal();
    short right = sensors->getCliffRightSignal();
    short frontLeft = sensors->getCliffFrontLeftSignal();
    short frontRight = sensors->getCliffFrontRightSignal();
    bool leftWheel = sensors->getWheeldropLeft();
    bool rightWheel = sensors->getWheeldropRight();
    bool caster = sensors->getWheeldropCaster();

    bool cliff = (left == 0 && right == 0) || (frontLeft == 0 && frontRight == 0);
    bool wheelDrop = leftWheel || rightWheel || caster;

    if (cliff || wheelDrop ) {
      cliffCounter++;
      cout << left << " " << right << " " << frontLeft << " " << frontRight << endl;
    }
    else {
      cliffCounter = 0;
    }

    if (cliffCounter > 50) {
      if (cliff) {
	      cout << "Cliff detected " << endl;
      } else {
	      cout << "wheel drop detected" << endl;
      }
      error = true;
      stopAndPlaySong(stream_mutex, robot);
    } else if (error) {
      error = false;
      startAgain(stream_mutex, robot, speed);
    }
  }
  return NULL;
}
