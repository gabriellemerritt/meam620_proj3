#ifndef _TRAJ_GEN_H_
#define _TRAJ_GEN_H_

// #include <math.h> 
// #include <stdio.h> 
#include "BebopDroneDecodeStream.h"


// typedef struct 
// {
// 	float trajStartTime; 
// 	float x_offset; 
// 	float y_offset; 
// 	float z_offset;
// 	float ax_des; 
// 	float ay_des; 
// 	float az_des;
//     float vx_des;
//     float vy_des;
//     float vz_des;
//     float x_des;
//     float y_des;
//     float z_des;
// } TRAJECTORY_t;

// typedef struct 
// {
// 	float coef_x[6]; 
// 	float coef_y[6]; 
// 	float coef_z[6];
// 	float traj_time;  

// }COEFF_t; 

void followTrajectory(TRAJECTORY_t traj, void *customData);   
// void genTrajectory (TRAJECTORY_t traj, COEFF_t coef, void *customData, float t); 
int readTrajectory (const char* file_name, int line_number, void *customData); 
void generateTrajectory(void *customData); 
 







#endif //_TRAJ_GEN_H_ // 