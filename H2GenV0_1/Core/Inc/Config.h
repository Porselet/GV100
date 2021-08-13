#ifndef __config_h
#define __config_h

#define MAX_TEMP 70.0
#define MIN_TEMP 5.0

#define MAX_PRESS 2.5
#define MAX_WQ 20.0
#define MAX_FLOW 146

#define WARNING_WQ 9.0
#define WARNING_WQ_TIME 30

#define MAX_VOLTAGE 420
#define MAX_AMPERAGE 300

#define LEAK_CHECK_TIME1 75000 
#define LEAK_CHECK_AMPERAGE1 100
#define LEAK_CHECK_PRESS1 0.8

#define LEAK_CHECK_TIME2 35000
#define LEAK_CHECK_AMPERAGE2 20
#define LEAK_CHECK_PRESS2 0.9

#define COUNT_CELL 14
//множитель, связывающий ток и выход водорода
#define MULT_AMP_TO_FLOW (COUNT_CELL * 0.418)


#define LEAK_CHECK_PRESS_IN_NORMAL_WORK 0.7

//Настройки клапана перелива
#define MIN_PRESS_TO_POUR 0.5
#define TIME_TO_POUR 2000

//Настройки насоса
#define TIME_PUMP_ON 5000
#define TIME_PUMP_OFF 10000
#define PUMP_PWM 30 /*  */

//config PID coefficients
#define P_COEFFICIENT 1
#define I_COEFFICIENT 0.1
#define ISUMMIN -0.3
#define ISUMMAX 0.3

#define PID_OFFSET 0
#define PRESS_SETPOINT 2.0
#define PID_MULT (1000.0/PRESS_SETPOINT)


//скорость приращения тока
#define SPEED_PS 2
#define SPEED_DELAY 100

// странички с авариями
#define ALARM_PAGE_PRESS "page HighPress"
#define ALARM_PAGE_LEAK_H2 "page Leak_H2"
#define ALARM_PAGE_LOW_LEVEL_O2 "page Low_level_O2"
#define ALARM_PAGE_HIGH_LEVEL_O2 "page High_level_O2"
#define ALARM_PAGE_LOW_LEVEL_H2 "page Low_level_H2"
#define ALARM_PAGE_HIGH_LEVEL_H2 "page High_level_H2"
#define ALARM_PAGE_LOW_TEMP "page Low_temp"
#define ALARM_PAGE_HIGH_TEMP "page High_temp"
#define ALARM_PAGE_WQ "page Bad_water"
#define ALARM_PAGE_LOW_VOLTAGE "page Low_voltage"
#define WARNING_PAGE_WQ "page Warning_water"

//const char* AlarmPagePress = ALARM_PAGE_PRESS;

#endif //__config_h