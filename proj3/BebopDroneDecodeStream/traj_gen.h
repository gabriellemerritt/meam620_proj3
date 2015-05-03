#ifndef _TRAJ_GEN_H_
#define _TRAJ_GEN_H_

#include "BebopDroneDecodeStream.h"
#include "ihm.h"

void followTrajectory(TRAJECTORY_t traj, void *customData);   

void genTrajectory (void *customData);

int readTrajectory (const char* file_name, int line_number, void *customData); 

int lengthTrajectory(const char* file_name); 

void generateTrajectory(void *customData); 

void runTrajectory(eIHM_INPUT_EVENT event, void *customData, const char* file_name);

 

#endif //_TRAJ_GEN_H_ // 