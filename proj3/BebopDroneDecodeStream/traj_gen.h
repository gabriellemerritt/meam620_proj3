#ifndef _TRAJ_GEN_H_
#define _TRAJ_GEN_H_

#include <math.h> 
#include <stdio.h>

typedef struct 
{
    float vx_des;
    float vy_des;
    float vz_des;
    float x_des;
    float y_des;
    float z_des;
} TRAJECTORY_t;

typedef struct 
{
	float coef_x[6]; 
	float coef_y[6]; 
	float coef_z[6];
	float traj_time;  
	// float coef_vx[6]; 
	// float coef_vy[6]; 
	// float coef_vz[6]; 
	// float coef_ax[6]; 
	// float coef_ay[6]; 
	// float coef_az[6]; 

}COEFF_t; 

void followTrajectory(TRAJECTORY_t traj, void *customData);   
void genTrajectory (COEFF_t coef, void *customData); 
void readTrajectory (COEFF_t coef, char* file_name); 
 







#endif //_TRAJ_GEN_H_ // 