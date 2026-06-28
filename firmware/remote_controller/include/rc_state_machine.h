#ifndef RC_STATE_H
#define RC_STATE_H

#include <stdbool.h>

typedef enum{
    RC_STATE_IDLE = 0,
    RC_STATE_ARMED = 1
} rc_states;

extern rc_states current_rc_state;

int rc_state_init(void); 
bool rc_check_arm(void);

#endif