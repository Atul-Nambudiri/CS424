#include "RobotSafety.hh"

void RobotSafety::stopAndPlaySong(pthread_mutex_t * stream_mutex, Create * robot, bool * stopped) {
        //stop robot and play song
        pthread_mutex_lock(stream_mutex);
        robot->sendDriveCommand(0, Create::DRIVE_STRAIGHT);			
        *stopped = true;
        robot->sendPlaySongCommand(0);
        sleep(1);
        robot->sendPlaySongCommand(0);
        pthread_mutex_unlock(stream_mutex);
}

void RobotSafety::startAgain(pthread_mutex_t * stream_mutex, Create * robot, int speed, int speed_diff) {
    pthread_mutex_lock(stream_mutex);
    robot->sendDriveDirectCommand(speed + speed_diff, speed);	
	//robot->sendDriveCommand(speed, Create::DRIVE_STRAIGHT);  
    pthread_mutex_unlock(stream_mutex);
}

void * RobotSafety::overcurrent(void * args) {
    RobotSafetyStruct * info = (RobotSafetyStruct *) args;
    pthread_mutex_t * stream_mutex = info->stream_mutex;
    Create * robot = info->robot;
    int speed = info->speed;
    int speed_diff = info->speed_diff;
    bool * stopped = info->stopped;
    pthread_cond_t * cv = info->cv;
  
    bool error = false;
    cout << "Overcurrent Thread Starting" << endl;
    while (1) {
        //Check for overcurrent in either wheel
        pthread_mutex_lock(stream_mutex);
        bool left = robot->leftWheelOvercurrent();
        bool right = robot->rightWheelOvercurrent();
        pthread_mutex_unlock(stream_mutex);

        if (left && right) {

      	    cout << "overcurrent detected" << endl;
	        error = true;

	        this_thread::sleep_for(chrono::milliseconds(1000));

	        //check again
	        pthread_mutex_lock(stream_mutex);
    	    bool left = robot->leftWheelOvercurrent();
    	    bool right = robot->rightWheelOvercurrent();
    	    pthread_mutex_unlock(stream_mutex);

	        if (left && right) {
	            error = true;
	    
	            cout << "overcurrent confirmed" << endl;

	            //stop robot and play song
	            stopAndPlaySong(stream_mutex, robot, stopped);
	        }
        } else if (error) {
            *stopped = false;
            error = false;
            pthread_cond_broadcast(cv);
	    }
    }

    return NULL;
}

void * RobotSafety::cliffWheelDrop(void * args) {
    RobotSafetyStruct * info = (RobotSafetyStruct *) args;
    pthread_mutex_t * stream_mutex = info->stream_mutex;
    Create * robot = info->robot;
    int speed = info->speed;
    int speed_diff = info->speed_diff;
    bool * stopped = info->stopped;
    pthread_cond_t * cv = info->cv;

    this_thread::sleep_for(chrono::milliseconds(1000)); // So it doens't beep in the beginning
    
    bool error = false;
    cout << "Cliff Thread Starting" << endl;
    cout << "Wheel Drop Thread Starting" << endl;
    while (1) {
	    pthread_mutex_lock(stream_mutex);
	    short left = robot->cliffLeftSignal();
	    short right = robot->cliffRightSignal();
	    short frontLeft = robot->cliffFrontLeftSignal();
	    short frontRight = robot->cliffFrontRightSignal();
        bool leftWheel = robot->wheeldropLeft();
        bool rightWheel = robot->wheeldropRight();
        bool caster = robot->wheeldropCaster();
	    pthread_mutex_unlock(stream_mutex);

	    if (left < 100 || right < 100 || frontLeft < 100 || frontRight < 100 || leftWheel || rightWheel || caster) {
		    cout << "cliff detected or wheel drop detected" << endl;
            error = true;
            stopAndPlaySong(stream_mutex, robot, stopped);
	    } else if (error) {
            *stopped = false;
            error = false;
            pthread_cond_broadcast(cv);
        }

    }
    return NULL;
}
