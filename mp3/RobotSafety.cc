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

void RobotSafety::startAgain(pthread_mutex_t * stream_mutex, Create * robot, int speed) { //This is never called
        pthread_mutex_lock(stream_mutex);
        robot->sendDriveCommand(speed, Create::DRIVE_STRAIGHT);  
        pthread_mutex_unlock(stream_mutex);
}

void * RobotSafety::overcurrent(void * args) {
        RobotSafetyStruct * info = (RobotSafetyStruct *) args;
        pthread_mutex_t * stream_mutex = info->stream_mutex;
        Create * robot = info->robot;
        int speed = info->speed;
        bool * stopped = info->stopped;
        bool * moving = info->moving;
        pthread_cond_t * cv = info->cv;

        bool error = false;
        cout << "Overcurrent Thread Starting" << endl;
        pthread_mutex_lock(&stream_mutex);
        bool local_moving = &moving;
        pthread_mutex_unlock(&stream_mutex);
        while (local_moving) {
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
                        startAgain(stream_mutex, robot, speed);
                }
                pthread_mutex_lock(&stream_mutex);
                local_moving = &moving;
                pthread_mutex_unlock(&stream_mutex);
        }

        return NULL;
}

void * RobotSafety::cliffWheelDrop(void * args) {
        RobotSafetyStruct * info = (RobotSafetyStruct *) args;
        pthread_mutex_t * stream_mutex = info->stream_mutex;
        Create * robot = info->robot;
        int speed = info->speed;
        bool * stopped = info->stopped;
        pthread_cond_t * cv = info->cv;

        this_thread::sleep_for(chrono::milliseconds(1000)); // So it doens't beep in the beginning

        int cliffCounter = 0;

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

                bool cliff = (left == 0 && right == 0) || (frontLeft == 0 && frontRight == 0);
                bool wheelDrop = leftWheel || rightWheel || caster;

                if (cliff || wheelDrop ) {
                        cliffCounter++;
                        cout << left << " " << right << " " << frontLeft << " " << frontRight << endl;
                }
                else {
                        cliffCounter = 0;
                }

                if (cliffCounter > 10) {
                        if (cliff) {
                                cout << "Cliff detected " << endl;
                        } else {
                                cout << "wheel drop detected" << endl;
                        }
                        error = true;
                        stopAndPlaySong(stream_mutex, robot, stopped);
                } else if (error) {
                        *stopped = false;
                        error = false;
                        startAgain(stream_mutex, robot, speed);
                }

        }
        return NULL;
}
