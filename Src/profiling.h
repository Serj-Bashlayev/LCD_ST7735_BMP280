#ifndef _PROFILING_H
#define _PROFILING_H

#include <stdint.h>
#include "stm32f3xx_hal.h"

void profiling_start(const char *profile_name); 
void profiling_event(const char *event);
void profiling_stop(void);

#endif // _PROFILING_H

