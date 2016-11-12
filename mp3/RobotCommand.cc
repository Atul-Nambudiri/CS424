#include "RobotCommand.hh"

void * executeCommands(void * args) {
  while (true) {
    RobotCommand * queue = (RobotCommand *) args;
    ComSig command = queue->pull();
    pthread_mutex_lock(queue->s_m);
    command(NULL);
    pthread_mutex_unlock(queue->s_m);
  }
}

RobotCommand::RobotCommand(pthread_mutex_t * stream_mutex) {
  head = NULL;
  tail = head;
  size = 0;
  s_m = stream_mutex;

  pthread_mutex_init(&m, NULL);
  pthread_cond_init(&cv, NULL);
}

RobotCommand::~RobotCommand() {
  pthread_mutex_destroy(&m);
  pthread_cond_destroy(&cv);
  while (head != NULL) {
    queue_node_t *tmp = head->next;
    free(head);
    head = tmp;
  }
}

void push(ComSig) {
  pthread_mutex_lock(&m);
  queue_node_t * node = malloc(sizeof(queue_node_t));
  node->commmand = command;
  if (tail != NULL) {
    tail->next = node;
  } else {
    head = node;
  }
  tail = node;
  size++;
  pthread_cond_signal(&cv);
  pthread_mutex_unlock(&m);
}

ComSig pull() {
  pthread_mutex_lock(&m);
  while (size == 0) {
    pthread_cond_wait(&cv, &m);
  }
  queue_node_t * tmp = head;
  head = head->next;
  commSig = tmp->command;
  free(tmp);
  tmp = NULL;
  size--;
  if (size == 0) {
    tail = NULL;
  }
  pthread_mutex_unlock(&m);
  return ret;
}
