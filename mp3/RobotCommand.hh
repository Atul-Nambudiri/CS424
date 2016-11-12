#ifndef ROBOTCOMMAND_HH
#define ROBOTCOMMAND_HH

#include <pthread.h>
#include <stdlib.h>

class RobotCommand {

  typedef void* (* ComSig)(void*param);

public:

  RobotCommand(pthread_mutex_t * stream_mutex);
  ~RobotCommand();
  static void * executeCommands(void * args);
  void push(ComSig);
  ComSig pull();

private:

  typedef struct queue_node_t {
    struct queue_node_t *next;
    ComSig command;
  } queue_node_t;

  queue_node_t *head, *tail;
  int size;
  pthread_mutex_t * s_m;
  pthread_mutex_t m;
  pthread_cond_t cv;

};
#endif
