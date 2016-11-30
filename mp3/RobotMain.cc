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
#include "RobotVision.hh"
#include "RobotSensors.hh"
#include "irobot-create.hh"
#include "RobotSafety.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

pthread_mutex_t stream_mutex;
pthread_cond_t condition_wait;
int speed = 140; // 50
bool turning = false;
bool right_turning = false;
int correctionCount = 1; // 2000 when by itself works perfect
int current_wall_signal = 0;
int prev_wall_signal = 0;
volatile bool timeout = false; /* Need volatile because changed in sig handler */
bool moving = true;

RobotSensors * sensorCache;

void sig_handler(int signum) {
  timeout = true;
}

void setTurning(bool turning1) {
  turning = turning1;
  if(!turning1) {
    pthread_cond_broadcast(&condition_wait);
  }
}
void moveRobot(Create & robot, short v, short r){
  pthread_mutex_lock(&stream_mutex);
  robot.sendDriveCommand(v,r);
  pthread_mutex_unlock(&stream_mutex);
}

void moveRobot(Create& robot, short v, Create::DriveCommand dc){
  pthread_mutex_lock(&stream_mutex);
  robot.sendDriveCommand(v, dc);
  pthread_mutex_unlock(&stream_mutex);    
}

void moveCounterClockwise(Create& robot){
  setTurning(true);
  moveRobot(robot, 70, Create::DRIVE_INPLACE_COUNTERCLOCKWISE); // 30
  setTurning(false);
  
  int local_prev_wall_signal = sensorCache->getWallSignal();
  int current_signal = sensorCache->getWallSignal();

  setTurning(true);
  cout << "start" << endl;
  int diff = sensorCache->getWallSignal() - (sensorCache->getPrevWallSignal() - 10);
  int prevdiff;
  while(diff > 5){ // current_signal > local_prev_wal_signal - 4
    prevdiff = diff;
    diff= sensorCache->getWallSignal() - (sensorCache->getPrevWallSignal() - 10);
    if (diff != prevdiff) {
      //      cout << "difference " << diff << endl;
    }
  }
  cout << "done" << endl;
  moveRobot(robot, 0, Create::DRIVE_STRAIGHT);
  setTurning(false);
  cout << "Turned All the way" << endl;
}

void moveClockwise(Create& robot){
  //robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  //robot.sendWaitAngleCommand(35);
  setTurning(true);
  moveRobot(robot, speed, -140);
  setTurning(true);
  // robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
  while(!sensorCache->getBumpLeft() && !sensorCache->getBumpRight() && (sensorCache->getWallSignal() < 120)) { }
  moveRobot(robot, -60, Create::DRIVE_STRAIGHT);
  this_thread::sleep_for(chrono::milliseconds(600));
  moveRobot(robot, 0, Create::DRIVE_STRAIGHT);       
  this_thread::sleep_for(chrono::milliseconds(300));
  cout << "Reached Wall" << endl;
  moveCounterClockwise(robot);
  moveRobot(robot, 0, Create::DRIVE_STRAIGHT); 
}

void setRobotTurnAngle(Create& robot, short angle){
  pthread_mutex_lock(&stream_mutex);
  robot.sendWaitAngleCommand(angle);
  pthread_mutex_unlock(&stream_mutex);
}

void turnLeft(Create& robot, RobotVision& vision) {
  setTurning(true);
  cout << "Reached Wall. Need to turn left" << endl;
  moveRobot(robot, -80, Create::DRIVE_STRAIGHT);
  this_thread::sleep_for(chrono::milliseconds(500));
  setTurning(true);
  moveRobot(robot, 50, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  setTurning(false);
  this_thread::sleep_for(chrono::milliseconds(1500));
             
  //setRobotTurnAngle(robot, 30); // This puts it in infinite spin
             
  moveRobot(robot, speed, Create::DRIVE_STRAIGHT);
  bool bump = sensorCache->getBumpRight() || sensorCache->getBumpLeft();
  
  while(!bump) {
    bump = sensorCache->getBumpRight() || sensorCache->getBumpLeft();
  }

  moveRobot(robot, -80, Create::DRIVE_STRAIGHT);

  this_thread::sleep_for(chrono::milliseconds(200));

  moveRobot(robot, 0, Create::DRIVE_STRAIGHT);
  vision.addNewWaypoint(speed);
  
  moveCounterClockwise(robot);
  
  prev_wall_signal = sensorCache->getWallSignal();
  vision.updateDirectionVector(); // Does this need to be 90 or -90?
  moveRobot(robot, speed, Create::DRIVE_STRAIGHT);
  cout << "Done turning left" << endl;
  setTurning(false);
}

void turnRight(Create& robot, RobotVision& vision) {
  cout << "Need to turn right" << endl;
  this_thread::sleep_for(chrono::milliseconds(1000));
  setTurning(true);
  moveRobot(robot, 0, Create::DRIVE_STRAIGHT);

  moveClockwise(robot);
  //  prev_wall_signal = robot.wallSignal();
  vision.addNewWaypoint(speed);
  vision.updateDirectionVector(); // Does this need to be 90 or -90?
  moveRobot(robot, speed, Create::DRIVE_STRAIGHT);
  setTurning(false);
}

void * mainThread(void * args) {
  RobotSafetyStruct * info = (RobotSafetyStruct *) args;
  Create robot = *(info->robot);

  this_thread::sleep_for(chrono::milliseconds(1000));
  bool bump = false;

  moveRobot(robot, speed, Create::DRIVE_STRAIGHT);
  cout << "Sent Drive Command" << endl;
 
  while(!bump) {
    bump = sensorCache->getBumpRight() || sensorCache->getBumpLeft();
  }

  moveRobot(robot, -80, Create::DRIVE_STRAIGHT);

  this_thread::sleep_for(chrono::milliseconds(200));    

  moveRobot(robot, 0, Create::DRIVE_STRAIGHT);

  cout << "Reached Wall" << endl;

  moveCounterClockwise(robot);

  moveRobot(robot, 0, Create::DRIVE_STRAIGHT);

  prev_wall_signal = sensorCache->getWallSignal();
  moveRobot(robot, speed, Create::DRIVE_STRAIGHT);
  
  int loop_counter = 0;
  int right_turn_counter = 0;
  RobotVision vision;
  while (true) { 
    // Need to turn left
    if(sensorCache->getBumpLeft() && sensorCache->getBumpRight()) {
      turnLeft(robot, vision);
    }

    //Need to turn right
    if(sensorCache->getWallSignal() == 0) {
      right_turn_counter ++;
      //if(right_turn_counter > 20000) { // Took if statement out because it will just do correction around corner
        turnRight(robot, vision);
        right_turn_counter = 0;
    //}
    }
    else {
      right_turn_counter = 0;
    }

    if(loop_counter % correctionCount == 0) {
      //Straighten out the movement - too close
      current_wall_signal = sensorCache->getWallSignal();
      if(current_wall_signal >= 70) {
        moveRobot(robot, speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
        this_thread::sleep_for(chrono::milliseconds(20));
        moveRobot(robot, speed, Create::DRIVE_STRAIGHT); 
      }
      if(current_wall_signal <= 10) {
        moveRobot(robot, speed, Create::DRIVE_INPLACE_CLOCKWISE);
        this_thread::sleep_for(chrono::milliseconds(20));
        moveRobot(robot, speed, Create::DRIVE_STRAIGHT); 
      }
    }
      
    int local_play = sensorCache->getPlayButton();
    loop_counter ++;

    if(local_play || timeout) {
      pthread_mutex_lock(&stream_mutex);
      robot.sendLedCommand (Create::LED_ADVANCE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
      pthread_mutex_unlock(&stream_mutex);    
      cout << "Play Pressed" << endl;
      vision.drawContourMap();
      moving = false;
      break;     
    }
  }
  moveRobot(robot, 0, Create::DRIVE_INPLACE_CLOCKWISE);
  cout << "Done Moving" << endl;
  return NULL;
}
int main(int argc, char** argv)
{
  pid_t child = fork();

  if (child == 0) {
    execvp(argv[1], argv+1);
    perror("Fork()");
    exit(1);
  }
  char serial_loc[] = "/dev/ttyUSB0";

  //set up signal handler to handle SIGALRM
  struct sigaction action;
  memset(&action, 0x00, sizeof (action));
  action.sa_handler = sig_handler;
  if (sigaction(SIGALRM, &action, NULL) != 0) {
    perror("sigaction");
    return 1;
  }
  //setup sig event so timer generates SIGALRM
  struct sigevent timer_event;
  memset(&timer_event, 0x00, sizeof (timer_event));
  timer_event.sigev_notify = SIGEV_SIGNAL;
  timer_event.sigev_signo = SIGALRM;
  //create timer
  timer_t timer;
  if (timer_create(CLOCK_MONOTONIC, &timer_event, &timer) != 0) {
    perror("timer_create");
    return 1;
  }
  //set up timer to go off after 2 minutes, do not repeat
  struct itimerspec timer_time;
  struct timespec res, zero_res;
  res.tv_sec = (time_t) 200; //stop 5 seconds before 2 minutes
  res.tv_nsec = 0;
  zero_res.tv_sec = 0;
  zero_res.tv_nsec = 0;
  timer_time.it_interval = zero_res; //no repeat
  timer_time.it_value = res;
  //schedule timer
  if (timer_settime(timer, 0, &timer_time, NULL) != 0) {
    perror("timer_settime");
    return 1;
  }
  cout << "timer started" << endl;


  try
    {
      SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
      cout << "Opened Serial Stream to" << serial_loc << endl;
      this_thread::sleep_for(chrono::milliseconds(1000));
      Create robot(stream);
      cout << "test" << endl;
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
      sensors.push_back (Create::SENSOR_BUTTONS);
      sensors.push_back(Create::SENSOR_ANGLE);
      //sensors.push_back(Create::SENSOR_GROUP_0);
      //sensors.push_back(Create::SENSOR_GROUP_3);

      robot.sendStreamCommand (sensors);
      cout << "Sent Stream Command" << endl;
        
      pthread_cond_init(&condition_wait, NULL);
      pthread_mutex_init(&stream_mutex, NULL);

      sensorCache = new RobotSensors(robot, &stream_mutex);

      //Start the Robot safety threads
      pthread_t overcurrent_thread, cliff_wheelDrop_thread, objectID, main_thread, sensor_thread;
      pthread_attr_t cliffAttr, mainAttr, visionAttr, OCAttr, sensorAttr;
      struct sched_param cliffParam, mainParam, visionParam, OCParam, sensorParam;
   
      if (pthread_attr_init(&cliffAttr) != 0) {
    perror("attr_init cliffAttr");
      }
      if (pthread_attr_init(&mainAttr) != 0) {
    perror("attr_init mainAttr");
      }
      if (pthread_attr_init(&visionAttr) != 0) {
    perror("attr_init visionAttr");
      }
      if (pthread_attr_init(&OCAttr) != 0) {
    perror("attr_init OCAttr");
      }
      if (pthread_attr_init(&sensorAttr) != 0) {
    perror("attr_init sensorAttr");
      }
    
      if (pthread_attr_setinheritsched(&cliffAttr, PTHREAD_EXPLICIT_SCHED) != 0) {
    perror("attr_setinheritsched cliffAttr");
      }
      if (pthread_attr_setinheritsched(&visionAttr, PTHREAD_EXPLICIT_SCHED) != 0) {  
    perror("attr_setinheritsched visionAttr");
      }
      if (pthread_attr_setinheritsched(&OCAttr, PTHREAD_EXPLICIT_SCHED) != 0) {  
    perror("attr_setinheritsched OCAttr");
      }
      if (pthread_attr_setinheritsched(&mainAttr, PTHREAD_EXPLICIT_SCHED) != 0) { 
    perror("attr_setinheritsched mainAttr");
      }
      if (pthread_attr_setinheritsched(&sensorAttr, PTHREAD_EXPLICIT_SCHED) != 0) { 
    perror("attr_setinheritsched sensorAttr");
      }

      if (pthread_attr_getschedparam(&mainAttr, &mainParam) != 0) {
    perror("attr_getschedparam mainAttr");
      }
      if (pthread_attr_getschedparam(&OCAttr, &OCParam) != 0) {
    perror("attr_getschedparam OCAttr");
      }
      if (pthread_attr_getschedparam(&cliffAttr, &cliffParam) != 0) {
    perror("attr_getschedparam cliffAttr");
      }
      if (pthread_attr_getschedparam(&visionAttr, &visionParam) != 0) {
    perror("attr_getschedparam visionAttr");
      }
      if (pthread_attr_getschedparam(&sensorAttr, &sensorParam) != 0) {
    perror("attr_getschedparam sensorAttr");
      }
        
      mainParam.sched_priority = 95;
      OCParam.sched_priority = 18;
      cliffParam.sched_priority = 60;
      visionParam.sched_priority = 1;
      sensorParam.sched_priority = 90;

      if (pthread_attr_setschedpolicy(&mainAttr, SCHED_FIFO) != 0) {
    perror("attr_setschedpolicy mainAttr");
      }
      if (pthread_attr_setschedpolicy(&OCAttr, SCHED_FIFO) != 0) {
    perror("attr_setschedpolicy OCAttr");
      }
      if (pthread_attr_setschedpolicy(&cliffAttr, SCHED_FIFO) != 0) {
    perror("attr_setschedpolicy cliffAttr");
      }
      if (pthread_attr_setschedpolicy(&visionAttr, SCHED_FIFO) != 0) {
    perror("attr_setschedpolicy visionAttr");
      }
      if (pthread_attr_setschedpolicy(&sensorAttr, SCHED_FIFO) != 0) {
    perror("attr_setschedpolicy sensorAttr");
      }

      if (pthread_attr_setschedparam(&mainAttr, &mainParam) != 0) {
    perror("attr_setschedparam visionAttr");
      }
      if (pthread_attr_setschedparam(&OCAttr, &OCParam) != 0) { 
    perror("attr_setschedparam OCAttr");
      }
      if (pthread_attr_setschedparam(&cliffAttr, &cliffParam) != 0) {
    perror("attr_setschedparam cliffAttr");
      }
      if (pthread_attr_setschedparam(&visionAttr, &visionParam) != 0) {
    perror("attr_setschedparam visionAttr");
      }
      if (pthread_attr_setschedparam(&sensorAttr, &sensorParam) != 0) {
    perror("attr_setschedparam sensorAttr");
      }

      RobotSafetyStruct thread_info;
      thread_info.speed = speed;
      thread_info.robot = &robot;
      thread_info.stream_mutex = &stream_mutex;
      thread_info.turning = &turning;
      thread_info.cv = &condition_wait;
      thread_info.timeout = &timeout;
      thread_info.moving = &moving;
      thread_info.sensorCache = sensorCache;

      if (pthread_create(&main_thread, &mainAttr, &mainThread, &thread_info) != 0) {
    perror("pthread_create main_thread");
    return -1;
      }
     if (pthread_create(&overcurrent_thread, &OCAttr, &RobotSafety::overcurrent, &thread_info) != 0) {
     perror("pthread_create overcurrent_thread");
     return -1;
     }
     if (pthread_create(&cliff_wheelDrop_thread, &cliffAttr, &RobotSafety::cliffWheelDrop, &thread_info) != 0) {
     perror("pthread_create cliff_wheelDrop_thread");
     return -1;
     }
     if (pthread_create(&objectID, &visionAttr, &RobotVision::objectIdentification, &thread_info) != 0) {
     perror("pthread_create objectID_thread");
     return -1;
     }      pthread_create(&sensor_thread, &sensorAttr, &RobotSensors::startUpdateValues, sensorCache);
      
      cout << "After Create" << endl;
      sleep(10);

      pthread_join(main_thread, NULL);
      pthread_join(overcurrent_thread, NULL);
      pthread_join(cliff_wheelDrop_thread, NULL);
      pthread_join(objectID, NULL);

      pthread_attr_destroy(&cliffAttr);
      pthread_attr_destroy(&visionAttr);
      pthread_attr_destroy(&OCAttr);
      pthread_attr_destroy(&mainAttr);
      //robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
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
  cout << "Hi" << endl;   
}
