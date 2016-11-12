#include <SerialStream.h>
#include <pthread.h>
#include <iostream>
#include <unistd.h>
#include "irobot-create.hh"

using namepsace iRobot;
using namespace LibSerial;
using namespace std;

pthread_mutex_t stream_mutex;

int main(int argc, char** argv) {


  try {

    /* Set up stream, create robot object, and set it to full mode */
    SerialStream stream("/dev/ttyUSB0", LibSerial::SerialStreamBuf::BAUD_57600);
    sleep(1);
    Create robot(stream);
    robot.sendFullCommand();
    sleep(1);

    /* Create thread */
    
  } catch (InvalidArgument& e) {
    cerr << e.what() << endl;
    return 3;
  } catch (CommandNotAvailable& e) {
    cerr << e.what() << endl;
    return 4;
  }
}
