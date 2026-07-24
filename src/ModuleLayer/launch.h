#ifndef __LAUNCH_H
#define __LAUNCH_H

#include "stdint.h"

typedef enum{
	L_LOCK =0,
//	L_INIT,
  L_UNLOCK,

}Launch_State_e;


typedef enum{
	SINGLE_SHOT,
	REPEAT_SHOT,

}Launch_Mode_e;


typedef struct Launch_Struct_t{
	Launch_State_e    state;
  	Launch_Mode_e     mode;
	
	uint8_t           shoot_level;
	uint8_t           shoot_lock;
	
	void (*init)(struct Launch_Struct_t* launch);
	void (*work)(struct Launch_Struct_t* launch);

}Launch_t;

extern  Launch_t  launch;
#endif

