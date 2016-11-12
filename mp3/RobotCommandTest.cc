#include <SerialStream.h>
#include <pthread.h>
#include <iostream>
#include <unistd.h>
#include "irobot-create.hh"
#include "RobotCommand.hh"

using namespace iRobot;
using namespace LibSerial;
using namespace std;

Create * robot;

void * driveStraight(void * args) {
  robot->sendDriveCommand(100, Create::DRIVE_STRAIGHT);
  return NULL;
}

int main(int argc, char** argv) {

  try {

    /* Set up stream, create robot object, and set it to full mode */
    SerialStream stream("/dev/ttyUSB0", LibSerial::SerialStreamBuf::BAUD_57600);
    sleep(1);
    robot = new Create(stream);
    robot.sendFullCommand();
    sleep(1);
    pthread_mutex_t stream_mutex;
    pthread_mutex_init(&stream_mutex);

    /* Create thread */
    pthread_t command_thread;
    RobotCommand::RobotCommand queue(&stream_mutex);
    pthread_create(&command_thread, NULL, RobotCommand::executeCommands, &queue);
    queue.push(driveStraight);
    pthread_join(command_thread, NULL);

    pthread_mutex_destroy(&stream_mutex);
    
  } catch (InvalidArgument& e) {
    cerr << e.what() << endl;
    return 3;
  } catch (CommandNotAvailable& e) {
    cerr << e.what() << endl;
    return 4;
  }
}
