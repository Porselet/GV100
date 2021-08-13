#ifndef __NEXTION_INTERFACE_H
#define __NEXTION_INTERFACE_H

#include "UARTs.h"

void SendToNextion(char * msg);
void NextionPoll(struct DataStruct * values);

#endif //__NEXTION_INTERFACE_H