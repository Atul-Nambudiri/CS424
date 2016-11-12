#include <SerialStream.h>
#include "irobot-create.hh"
#include "RobotSafety.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <raspicam/raspicam_cv.h>
#include "RobotVision.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

std::mutex stream_mutex;
int speed = 100;

void overcurrent(Create& robot) {
  cout << "Overcurrent Thread Starting" << endl;
  int count = 0;
  while (1) {
    //Check for overcurrent in either wheel
    stream_mutex.lock();
    bool left = robot.leftWheelOvercurrent();
    bool right = robot.rightWheelOvercurrent();
    stream_mutex.unlock();

    if (left || right) {
      count++;
      cout << "overcurrent detected" << endl;
      //stop robot
      if (count == 100){
	stream_mutex.lock();
	robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);			

	//play song
	robot.sendPlaySongCommand(0);
	sleep(1);
	robot.sendPlaySongCommand(0);
	  
	stream_mutex.unlock();
      }
    }
    else 
      count = 0;
  }
}

void moveCounterClockwise(Create& robot){
    int prev_wall_signal = robot.wallSignal();
    robot.sendDriveCommand(30, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
    int current_signal = robot.wallSignal();
    while(current_signal >= prev_wall_signal || current_signal - prev_wall_signal >= -4) {
            prev_wall_signal = current_signal;
            current_signal = robot.wallSignal();
    }

    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
}


void moveClockwise(Create& robot){
    robot.sendDriveCommand(35, Create::DRIVE_INPLACE_CLOCKWISE);
    robot.sendWaitAngleCommand(-100);

    robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
    while(!robot.bumpLeft() && !robot.bumpRight() && (robot.wallSignal() < 120)) {
        stream_mutex.unlock();
        stream_mutex.lock();
    }
    robot.sendDriveCommand(-40, Create::DRIVE_STRAIGHT);
    stream_mutex.unlock();
    this_thread::sleep_for(chrono::milliseconds(1000));
    stream_mutex.lock(); 
    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
    stream_mutex.unlock();          
    this_thread::sleep_for(chrono::milliseconds(500));
    cout << "Reached Wall" << endl;
    stream_mutex.lock();
    moveCounterClockwise(robot);
    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT); 
}

int main ()
{
  	char serial_loc[] = "/dev/ttyUSB0";

  	try
  	{
        cout << "Test" << endl;
    		// raspicam::RaspiCam_Cv Camera;
    		// cv::Mat rgb_image, bgr_image;
    		// if (!Camera.open()) {
      //   		cerr << "Error opening the camera" << endl;
      //   		return -1;
      // 	}
      	cout << "Opened Camera" << endl;
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
      	sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
      	sensors.push_back(Create::SENSOR_WALL_SIGNAL);
        sensors.push_back(Create::SENSOR_OVERCURRENTS);
      	//sensors.push_back (Create::SENSOR_BUTTONS);
        //sensors.push_back(Create::SENSOR_GROUP_0);
        //sensors.push_back(Create::SENSOR_GROUP_3);

      	robot.sendStreamCommand (sensors);
      	cout << "Sent Stream Command" << endl;
		
    		// Let's move towards the wall
    		robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
    		cout << "Sent Drive Command" << endl;
    		while(!robot.bumpLeft() && !robot.bumpRight()) {}
    		robot.sendDriveCommand(-50, Create::DRIVE_STRAIGHT);
    		this_thread::sleep_for(chrono::milliseconds(1000));		
    		robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);			
    		this_thread::sleep_for(chrono::milliseconds(500));
    		cout << "Reached Wall" << endl;
        moveCounterClockwise(robot);
        robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT); 

        //Start the overcurrent_thread
	//	std::thread overcurrent_thread(RobotSafety.overcurrent, std::ref(robot), ref(stream_mutex));
           
        cout << "Thread Started" << endl;
        stream_mutex.lock();
        int prev_wall_signal = robot.wallSignal();
        robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
        int local_play = robot.playButton();
        int loop_counter = 0;
        int right_turn_counter = 0;
        RobotVision vision;
		stream_mutex.unlock();
        while (!local_play) {	
            stream_mutex.lock();
            // Need to turn left
            if(robot.bumpLeft() && robot.bumpRight()) {
                cout << "Reached Wall. Need to turn left" << endl;
                robot.sendDriveCommand(-40, Create::DRIVE_STRAIGHT);
                stream_mutex.unlock();
                this_thread::sleep_for(chrono::milliseconds(1000)); 
                stream_mutex.lock();    
                robot.sendDriveCommand(30, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                robot.sendWaitAngleCommand(40);
                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                vision.addNewWaypoint(speed);
                moveCounterClockwise(robot);
                vision.updateDirectionVector();
                robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
            }

            //Need to turn right
            if(robot.wallSignal() == 0) {
                right_turn_counter ++;
                if(right_turn_counter > 20) {
                  cout << "Need to turn right" << endl;
                  stream_mutex.unlock();
                  this_thread::sleep_for(chrono::milliseconds(3300));
                  stream_mutex.lock();
                  robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                  vision.addNewWaypoint(speed);
                  moveClockwise(robot);
                  vision.addNewWaypoint(speed);
                  vision.updateDirectionVector();
                  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
                  right_turn_counter = 0;
                }
		    }
            stream_mutex.unlock();
            if(loop_counter % 3000 == 0) {
              //Straighten out the movement
              stream_mutex.lock();
              int current_wall_signal = robot.wallSignal();
              stream_mutex.unlock();
              cout << prev_wall_signal << " " << current_wall_signal << " " << loop_counter << endl;
              if(prev_wall_signal - current_wall_signal >= 4  || current_wall_signal == 0) {
                  cout << "Correcting Route - Too far" << endl;
                  stream_mutex.lock();
                  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_CLOCKWISE);
                  stream_mutex.unlock();
                  this_thread::sleep_for(chrono::milliseconds(200));
                  stream_mutex.lock();
                  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
                  stream_mutex.unlock();
              }
			  cout << "Middle" << endl;
              if(prev_wall_signal - current_wall_signal <= -4) {
                  cout << "Correcting Route - Too Close" << endl;
                  stream_mutex.lock();
                  robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                  stream_mutex.unlock();
                  this_thread::sleep_for(chrono::milliseconds(200));
                  stream_mutex.lock();
                  robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
                  stream_mutex.unlock();
              }
			  cout << "End" << endl;
              prev_wall_signal = current_wall_signal;
            }
            
            stream_mutex.lock();
            local_play = robot.playButton();
            stream_mutex.unlock();
            loop_counter ++;
            if(loop_counter % 10000 == 9999)
                vision.drawContourMap();
        }
		    robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
    	

		    //    		overcurrent_thread.join();
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
 
