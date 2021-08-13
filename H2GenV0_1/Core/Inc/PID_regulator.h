#ifndef __PID_REGULATOR_H
#define __PID_REGULATOR_H
#include "stdint.h"
#include "Config.h"
extern double P_component ;
extern double I_component ;


uint16_t PID_Calculated(double Feedback);




#endif // __PID_REGULATOR_H