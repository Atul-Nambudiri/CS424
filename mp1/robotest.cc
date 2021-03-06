#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace iRobot;
using namespace LibSerial;
using namespace std;

int handle_flag = 1;
int led_flag = 1;
std::mutex stream_mutex;

void handleWall(Create& robot) {
	short wallSignal, prevWallSignal = 0;
	int sleep_period = 300;
	
	stream_mutex.lock();
	int local_flag = handle_flag;
	stream_mutex.unlock();
	while(local_flag) {
		stream_mutex.lock();
		wallSignal = robot.wallSignal();
		stream_mutex.unlock();
		if (wallSignal > 0) {
			cout << "Wall signal " << wallSignal << endl;
			robot.sendPlaySongCommand(0);
			if(wallSignal > prevWallSignal + 10) {
				sleep_period /= 2;		//Lower Period
			}
			else if(wallSignal < prevWallSignal - 10) {
				sleep_period *= 2;		//Increase Period
				if(sleep_period > 300) {
					sleep_period = 300;	//No more than .5 second in between
				}
			}
		}
		else {
			sleep_period = 300;
		}
		prevWallSignal = wallSignal;
		this_thread::sleep_for(chrono::milliseconds(sleep_period));			
		stream_mutex.lock();
		local_flag = handle_flag;
		stream_mutex.unlock();
	}
}

void handleLED(Create& robot) {
	stream_mutex.lock();
	int local_flag = led_flag;
	stream_mutex.unlock();

	while (local_flag){
		stream_mutex.lock();
    	robot.sendLedCommand (Create::LED_PLAY, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL); // GGO
    	stream_mutex.unlock();	
		this_thread::sleep_for(chrono::milliseconds(200));
    	
		stream_mutex.lock();
    	robot.sendLedCommand (Create::LED_ALL, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF); //  OGG
    	stream_mutex.unlock();
		this_thread::sleep_for(chrono::milliseconds(200));
    	
		stream_mutex.lock();
    	robot.sendLedCommand (Create::LED_ADVANCE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL); // ROG
    	stream_mutex.unlock();
		this_thread::sleep_for(chrono::milliseconds(200));
    
    	stream_mutex.lock();
		robot.sendLedCommand (Create::LED_PLAY, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL); // RGO
    	stream_mutex.unlock();
		this_thread::sleep_for(chrono::milliseconds(200));
    
    	stream_mutex.lock();
		robot.sendLedCommand (Create::LED_ALL, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF); // OGG
    	stream_mutex.unlock();
		this_thread::sleep_for(chrono::milliseconds(200));
    
    	stream_mutex.lock();
		robot.sendLedCommand (Create::LED_ADVANCE, Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL); // GOG
    	stream_mutex.unlock();
		this_thread::sleep_for(chrono::milliseconds(200));
  		
		stream_mutex.lock();
		local_flag = led_flag;
		stream_mutex.unlock();

	}
	cout << "End" << endl;
}

int main ()
{
  char serial_loc[] = "/dev/ttyUSB0";

  try
  {
    raspicam::RaspiCam_Cv Camera;
    cv::Mat rgb_image, bgr_image;
    if (!Camera.open()) {
      cerr << "Error opening the camera" << endl;
      return -1;
    }
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
    sensors.push_back (Create::SENSOR_BUTTONS);

    robot.sendStreamCommand (sensors);
    cout << "Sent Stream Command" << endl;
    
	//Start the wall signal thread
	std::thread wall_thread(handleWall, std::ref(robot));
	// Let's turn!
    int speed = 287;
    int ledColor = Create::LED_COLOR_GREEN;
    stream_mutex.lock();
	robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
    robot.sendLedCommand (Create::LED_PLAY, 0, 0);
    stream_mutex.unlock();
	cout << "Sent Drive Command" << endl;
	
	stream_mutex.lock();
	int local_play = robot.playButton();
    stream_mutex.unlock();
	while (!local_play)
    {	
		stream_mutex.lock();
		if (robot.bumpLeft () || robot.bumpRight ()) {
			stream_mutex.unlock();
			cout << "Bump !" << endl;
          	
			led_flag = 1;
          	std::thread led_thread(handleLED, std::ref(robot));

          	stream_mutex.lock();
			robot.sendDriveCommand(-165, Create::DRIVE_STRAIGHT);
        	stream_mutex.unlock();
			this_thread::sleep_for(chrono::milliseconds(2309));
          	stream_mutex.lock();
			robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			stream_mutex.unlock();			

			cout << "Before" << endl;
          	Camera.grab();
			cout << "After Grab" << endl;
          	Camera.retrieve (bgr_image);
			cout << "After Retrieve" << endl;
          	cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
			cout << "After cvt color" << endl;
          	cv::imwrite("irobot_image.jpg", rgb_image);
          	cout << "Taking photo" << endl;
			
			stream_mutex.lock();
          	led_flag = 0;
			stream_mutex.unlock();
			led_thread.join();

          	short angle = -(std::rand() % (121) + 120);
			cout << "Angle " << angle << endl;
        	stream_mutex.lock();
			robot.sendDriveCommand(50, Create::DRIVE_INPLACE_CLOCKWISE);
			stream_mutex.unlock();
			stream_mutex.lock();
			robot.sendWaitAngleCommand(angle);
			stream_mutex.unlock();
			stream_mutex.lock();
			robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT); 
			stream_mutex.unlock();
			stream_mutex.lock();
			robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT); 
			stream_mutex.unlock();
		} else {
			stream_mutex.unlock();
		}
		stream_mutex.lock();
		local_play = robot.playButton();
		stream_mutex.unlock();
	}
    stream_mutex.lock();
	handle_flag = 0;
	stream_mutex.unlock();
	wall_thread.join();
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

