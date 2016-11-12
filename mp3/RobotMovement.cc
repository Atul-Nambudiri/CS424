#include "RobotMovement.hh"

void setTurning(bool turning1) {
  turning = turning1;
  if(!turning1) {
    pthread_cond_broadcast(&condition_wait);
  }
}

void moveCounterClockwise(Create& robot){
  int local_prev_wall_signal = robot.wallSignal();
  setTurning(true);
  robot.sendDriveCommand(30, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  int current_signal = robot.wallSignal();
  while(current_signal > local_prev_wall_signal - 4){
    local_prev_wall_signal = current_signal;
    current_signal = robot.wallSignal();
  }
  setTurning(false);
  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  cout << "Turned All the way" << endl;
}


void moveClockwise(Create& robot){
  setTurning(true);
  robot.sendDriveCommand(55, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  robot.sendWaitAngleCommand(20);
  // setTurning(false);
  robot.sendDriveDirectCommand(speed + 90, speed);
  // robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
  while(!robot.bumpLeft() && !robot.bumpRight() && (robot.wallSignal() < 130)) {
    pthread_mutex_unlock(&stream_mutex);
    pthread_mutex_lock(&stream_mutex);
  }
  robot.sendDriveCommand(-60, Create::DRIVE_STRAIGHT);
  //   pthread_mutex_unlock(&stream_mutex);
  this_thread::sleep_for(chrono::milliseconds(600));
  // pthread_mutex_lock(&stream_mutex); 
  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  // pthread_mutex_unlock(&stream_mutex);          
  this_thread::sleep_for(chrono::milliseconds(300));
  cout << "Reached Wall" << endl;
  //   pthread_mutex_lock(&stream_mutex);
  moveCounterClockwise(robot);
  robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT); 
}

void correctLeft(Create& robot) {
  cout << "Correcting Route - Too Close" << endl;
  setTurning(true);
  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  while(prev_wall_signal - current_wall_signal <= -2) {
    current_wall_signal = robot.wallSignal();
  }
  setTurning(false);
  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
}

void correctRight(Create& robot) {
  cout << "Correcting Route - Too far" << endl;
  setTurning(true);
  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_CLOCKWISE);
  while(prev_wall_signal - current_wall_signal >= 2) {
    current_wall_signal = robot.wallSignal();
  }
  setTurning(false);
  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
}

void turnLeft(Create& robot, RobotVision& vision) {
  cout << "Reached Wall. Need to turn left" << endl;
  robot.sendDriveCommand(-30, Create::DRIVE_STRAIGHT);
  this_thread::sleep_for(chrono::milliseconds(500)); 
  setTurning(true);
  robot.sendDriveCommand(50, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  robot.sendWaitAngleCommand(75);
  setTurning(false);
  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  vision.addNewWaypoint(speed);
  moveCounterClockwise(robot);
  prev_wall_signal = robot.wallSignal();
  vision.updateDirectionVector(); // Does this need to be 90 or -90?
  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
  cout << "Done turning left" << endl;
}

void turnRight(Create& robot, RobotVision& vision) {
  cout << "Need to turn right" << endl;
  //pthread_mutex_unlock(&stream_mutex);
  // this_thread::sleep_for(chrono::milliseconds(4000));
  //pthread_mutex_lock(&stream_mutex);
  // robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  moveClockwise(robot);
  prev_wall_signal = robot.wallSignal();
  vision.addNewWaypoint(speed);
  vision.updateDirectionVector(); // Does this need to be 90 or -90?
  setTurning(false);
  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);

}

void * movementThread(void * args) {
  RobotSafetyStruct * info = (RobotSafetyStruct *) args;
  Create robot = *(info->robot);

  bool bump = false;

  pthread_mutex_lock(&stream_mutex);
  robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
  cout << "Sent Drive Command" << endl;
  
  while(!bump) {
    bump = robot.bumpRight() || robot.bumpLeft();
  }
  //pthread_mutex_lock(&stream_mutex);
  //setTurning(true);
  robot.sendDriveCommand(-80, Create::DRIVE_STRAIGHT);
  // pthread_mutex_unlock(&stream_mutex);
  this_thread::sleep_for(chrono::milliseconds(200));    
  // pthread_mutex_lock(&stream_mutex);
  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);      
  //setTurning(false);
  cout << "Reached Wall" << endl;

  moveCounterClockwise(robot);
  robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);

  prev_wall_signal = robot.wallSignal();
  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);

  int loop_counter = 0;
  int right_turn_counter = 0;
  RobotVision vision;
  pthread_mutex_unlock(&stream_mutex);
  while (true) { 
    pthread_mutex_lock(&stream_mutex);

    // Need to turn left
    if(robot.bumpLeft() && robot.bumpRight()) {
      turnLeft(robot, vision);
    }

    //Need to turn right
    if(robot.wallSignal() == 0) {
      right_turn_counter ++;
      if(right_turn_counter > 700) {
	turnRight(robot, vision);
	right_turn_counter = 0;
      }
    }
    if(loop_counter % correctionCount == 0) {
      //Straighten out the movement - too close
      current_wall_signal = robot.wallSignal();
      cout << prev_wall_signal << " " << current_wall_signal << " " << loop_counter << endl;
      if(prev_wall_signal - current_wall_signal <= -10) {
	correctLeft(robot);
      }
      else if(robot.bumpRight()) {
	setTurning(true);
	robot.sendDriveCommand(-80, Create::DRIVE_STRAIGHT);
	this_thread::sleep_for(chrono::milliseconds(100));
	correctLeft(robot);
      }

      if(prev_wall_signal - current_wall_signal >= 5  || current_wall_signal == 0) {
	correctRight(robot);
      }
    }
      
    int local_play = robot.playButton();
    pthread_mutex_unlock(&stream_mutex);
    loop_counter ++;

    if(local_play) {
      vision.drawContourMap();
      break;	 
    }
  }
  return NULL;
}
