#include "PID_regulator.h"
 double P_component = 0;
 double I_component = 0;
 
uint16_t PID_Calculated(double Feedback)
{
	double Result = 0;
	double Error = PRESS_SETPOINT - Feedback;
	P_component = P_COEFFICIENT * Error;
	I_component = I_component + I_COEFFICIENT * Error;
	if (I_component >  ISUMMAX) {I_component =  ISUMMAX;};
	if (I_component < ISUMMIN) {I_component = ISUMMIN;};
	Result = (P_component + I_component - PID_OFFSET) * PID_MULT;
	if (Result < 0) { Result = 0;}
	return (uint16_t)Result;
};