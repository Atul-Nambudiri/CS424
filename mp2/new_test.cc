#include <SerialStream.h>
#include "irobot-create.hh"
#include "RobotSafety.hh"
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <mutex>
#include <vector>
#include <raspicam/raspicam_cv.h>
#include "RobotVision.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

pthread_mutex_t stream_mutex;
pthread_cond_t condition_wait;
int speed = 50; // 50
int speed_diff = 70; // 70
int original_extra_angle = 20;
int current_extra_angle = original_extra_angle;
bool turning = false;
int correctionCount = 3000;
int current_wall_signal = 0;
int prev_wall_signal = 0;
bool stopped = false;

void setTurning(bool turning1) {
  turning = turning1;
  if(!turning1) {
    pthread_cond_broadcast(&condition_wait);
  }
}

void moveCounterClockwise(Create& robot){
  int local_prev_wall_signal = robot.wallSignal();
  setTurning(true);
  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  int current_signal = robot.wallSignal();
  while(current_signal > local_prev_wall_signal - 3){
    local_prev_wall_signal = current_signal;
    current_signal = robot.wallSignal();
  }
  setTurning(false);
  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  cout << "Turned All the way" << endl;
}

void correctLeft(Create& robot, float multiplier) {
  cout << "Correcting Route - Too Close" << endl;
  setTurning(true);
  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  
  //Move past parralel so that the robot is facing out
  robot.sendWaitAngleCommand((short) (current_extra_angle * multiplier));
  setTurning(false);
  robot.sendDriveDirectCommand(speed + speed_diff, speed);
  cout << "Corrected" << endl;
}

void turnLeft(Create& robot, RobotVision& vision) {
  cout << "Reached Wall. Need to turn left" << endl;
  robot.sendDriveCommand(-60, Create::DRIVE_STRAIGHT);
  this_thread::sleep_for(chrono::milliseconds(500)); 
  setTurning(true);
  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  robot.sendWaitAngleCommand(40);
  setTurning(false);
  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  vision.addNewWaypoint(speed);
  moveCounterClockwise(robot);
  prev_wall_signal = robot.wallSignal();
  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  
  //Move past parralel so that the robot is facing out
  robot.sendWaitAngleCommand((short) (current_extra_angle * 0.7));
  vision.updateDirectionVector();
  robot.sendDriveDirectCommand(speed + speed_diff, speed);
  cout << "Done turning left" << endl;
}

void * mainThread(void * args) {
  RobotSafetyStruct * info = (RobotSafetyStruct *) args;
  Create robot = *(info->robot);
  bool bump = false;
  
  pthread_mutex_lock(&stream_mutex);
  robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
  cout << "Sent Drive Command" << endl;
  
  while(!bump) {
    bump = robot.bumpRight() || robot.bumpLeft();
  }

  robot.sendDriveCommand(-100, Create::DRIVE_STRAIGHT);

  this_thread::sleep_for(chrono::milliseconds(600));    
  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);      

  moveCounterClockwise(robot);
  prev_wall_signal = robot.wallSignal();
  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

  //Move past parralel so that the robot is facing out
  robot.sendWaitAngleCommand(current_extra_angle);
  robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

  robot.sendDriveDirectCommand(speed + speed_diff, speed);
  int local_play = robot.playButton();
  int loop_counter = 0;
  RobotVision vision;
  pthread_mutex_unlock(&stream_mutex);
  while (!local_play) { 
    pthread_mutex_lock(&stream_mutex);
    //Need to turn left
    if(robot.bumpLeft() && robot.bumpRight()) {
      turnLeft(robot, vision);
    }
    //cout << "Need to Correct. Wall Signal: " << robot.wallSignal() << " Prev: " << prev_wall_signal << "Loop Count: " << loop_counter << endl;
    if(loop_counter % correctionCount == 0) {
      //Straighten out the movement - too close
      current_wall_signal = robot.wallSignal();
      if(current_wall_signal >= 25) {
	correctLeft(robot, 1);
	current_extra_angle -= 5;
	if(current_extra_angle < 5) {
	  current_extra_angle = 5;
	}
      }
      else if (robot.bumpRight()) {
	robot.sendDriveCommand(-100, Create::DRIVE_STRAIGHT);
	this_thread::sleep_for(chrono::milliseconds(600)); 
	// Turn a little more if the robot bumps the wall
	correctLeft(robot, 1.5);
	current_extra_angle -= 5;
	if(current_extra_angle < 5) {
	  current_extra_angle = 5;
	}
      }
      else {
	current_extra_angle += 3;
	if(current_extra_angle > original_extra_angle) {
	  current_extra_angle = original_extra_angle;
	}
      }
    }
      
    local_play = robot.playButton();
    pthread_mutex_unlock(&stream_mutex);
    loop_counter ++;
    if(local_play) {
      vision.drawContourMap();
    }
  }
  return NULL;
}

int main (int argc, char** argv)
{
  pid_t child = fork();

  if (child == 0) {
    execvp(argv[1], argv+1);
    perror("Fork()");
    exit(1);
  }
  char serial_loc[] = "/dev/ttyUSB0";

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
      //sensors.push_back(Create::SENSOR_GROUP_0);
      //sensors.push_back(Create::SENSOR_GROUP_3);

      robot.sendStreamCommand (sensors);
      cout << "Sent Stream Command" << endl;
        
      pthread_cond_init(&condition_wait, NULL);
      pthread_mutex_init(&stream_mutex, NULL);

      //Start the Robot safety threads
      pthread_t overcurrent_thread, cliff_wheelDrop_thread, objectID, main_thread;
      pthread_attr_t cliffAttr, mainAttr, visionAttr, OCAttr;
      struct sched_param cliffParam, mainParam, visionParam, OCParam;
   
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
        
      mainParam.sched_priority = 95;
      OCParam.sched_priority = 10;
      cliffParam.sched_priority = 5;
      visionParam.sched_priority = 10;

      if (pthread_attr_setschedpolicy(&mainAttr, SCHED_RR) != 0) {
	perror("attr_setschedpolicy mainAttr");
      }
      if (pthread_attr_setschedpolicy(&OCAttr, SCHED_RR) != 0) {
	perror("attr_setschedpolicy OCAttr");
      }
      if (pthread_attr_setschedpolicy(&cliffAttr, SCHED_RR) != 0) {
	perror("attr_setschedpolicy cliffAttr");
      }
      if (pthread_attr_setschedpolicy(&visionAttr, SCHED_RR) != 0) {
	perror("attr_setschedpolicy visionAttr");
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

      RobotSafetyStruct thread_info;
      thread_info.speed = speed;
      thread_info.speed_diff = speed_diff;
      thread_info.robot = &robot;
      thread_info.stream_mutex = &stream_mutex;
      thread_info.turning = &turning;
      thread_info.cv = &condition_wait;
      thread_info.stopped = &stopped;

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
      }

      cout << "After Create" << endl;
      sleep(10);

      pthread_join(overcurrent_thread, NULL);
      pthread_join(main_thread, NULL);
      pthread_join(cliff_wheelDrop_thread, NULL);
      pthread_join(objectID, NULL);

      pthread_attr_destroy(&cliffAttr);
      pthread_attr_destroy(&visionAttr);
      pthread_attr_destroy(&OCAttr);
      pthread_attr_destroy(&mainAttr);
      robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
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
