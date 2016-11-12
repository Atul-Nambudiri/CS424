#ifndef ROBOTCOMMAND_HH
#define ROBOTCOMMAND_HH

#include <pthread.h>

class RobotCommand {

public:

  RobotCommand();
  static void * executeCommands(void * args);
  void push(void * (*command)(void *));
  void * (*commmand)(void *) pull();


private:

  typedef struct queue_node_t {
    struct queue_node_t *next;
    void * (*command)(void *);
  }

  queue_node_t *head, *tail;
  int size;
  pthread_mutex_t m;
  pthread_cond_t cv;

};
#endif
