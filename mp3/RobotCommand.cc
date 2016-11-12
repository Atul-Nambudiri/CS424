#include "RobotCommand.hh"

RobotCommand() {
}

void * executeCommands(void * args) {
  while (true) {
    RobotCommand * queue = (RobotCommand *) args;
    void * (*command)(void *) = queue->pull();
    command(NULL);
  }
}
