#ifndef PROJ_TYPES_H
#define PROJ_TYPES_H

#include "Arduino.h"

typedef struct{
  volatile int steps;
  volatile unsigned int hi_low;
  int dir;
  int enable;
  float tps;
  int period;

} stepper_t;


typedef enum { 
  ps_init,
  ps_sync,
  ps_stop,
  ps_sweep,
  ps_idle, 
  ps_test,
  ps_error
} fsm_state_t;

class fsm_t{
  public:
  uint32_t tes, tis;
  fsm_state_t state, prev_state;
  byte req_rotate, req_stop;
  
  void set_state(fsm_state_t new_state);
  void act(void);
  void progress(void);
};



#endif // PROJ_TYPES_H
