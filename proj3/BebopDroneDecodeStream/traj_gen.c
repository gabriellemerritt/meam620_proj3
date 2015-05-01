


#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream/ARStream.h>
#include <time.h> 
#include "BebopDroneDecodeStream.h"

#define KPX 50
#define KPY 50
#define KPZ 50

#define KDX 10
#define KDY 10
#define KDZ 10 

volatile int line_number; 


void followTrajectory(TRAJECTORY_t traj, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData;

    float ax_des, ay_des, yaw;
    yaw = deviceManager->flightStates.yaw_cur;
    //find desired acceleration in the x and y direction
    ax_des = KDX * (traj.vx_des - deviceManager->flightStates.vx_cur) + KPX * (traj.x_des - deviceManager->flightStates.x_cur);
    ay_des = KDY * (traj.vy_des - deviceManager->flightStates.vy_cur) + KPY * (traj.y_des - deviceManager->flightStates.y_cur);
    //put them into desired angle for the attitude controller
    deviceManager->dataPCMD.roll = 1/9.81*(ax_des*sin(yaw) - ay_des*cos(yaw));
    deviceManager->dataPCMD.pitch = 1/9.81*(ax_des*cos(yaw) + ay_des*sin(yaw));
    deviceManager->dataPCMD.gaz = traj.vz_des + KPZ * (traj.z_des - deviceManager->flightStates.z_cur);
}


void genTrajectory (TRAJECTORY_t traj, COEFF_t coef, void *customData, float t) 
{
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData;  
	// setting desired position 
	traj.x_des = coef.coef_x[0] + coef.coef_x[1]*t + coef.coef_x[2]*t^2 + coef.coef_x[3]*t^3 + coef.coef_x[4]*t^4 +coef.coef_x[5]*t^5; 
	traj.y_des = coef.coef_y[0] + coef.coef_y[1]*t + coef.coef_y[2]*t^2 + coef.coef_y[3]*t^3 + coef.coef_y[4]*t^4 +coef.coef_y[5]*t^5;
	traj.z_des = coef.coef_z[0] + coef.coef_z[1]*t + coef.coef_z[2]*t^2 + coef.coef_z[3]*t^3 + coef.coef_z[4]*t^4 +coef.coef_z[5]*t^5;

	// // setting desired velocity 
	traj.vx_des =  
	traj.vy_des = 
	traj.vz_des = 

	       //vel(2) = a(2,2) + 2*a(3,2)*t + 3*a(4,2)*t^2 + 4*a(5,2)*t^3 + 5*a(6,2)*t^4 ; 




}

// return 0 if succesfully read line  
// return 1  if can't open file  
// return 2  ??? 
int readTrajectory (COEFF_t coef, const char* file_name, line_number)
{ 
	FILE *traj_file; 
	traj_file = fopen(file_name, "r"); 
	if (!traj_file)
	{
		printf("Can't Open File \n"); 
		return 1; // file error 
	}
	char line[1024]; 
	count = 0; 
	char* pEnd;
    if(fgets(line, sizeof(line), traj_file) != NULL) /* read a line */
    {
        if (count == line_number)
        {
            sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
            &coef.traj_time, &coef.coef_x[0],&coef.coef_x[1], &coef.coef_x[2], &coef.coef_x[3],
             &coef.coef_x[4],&coef.coef_x[5], &coef.coef_y[0],&coef.coef_y[1],&coef.coef_y[2],
             &coef.coef_y[3],&coef.coef_y[4],&coef.coef_y[5], &coef.coef_z[0], &coef.coef_z[1],
             &coef.coef_z[2],&coef.coef_z[3],&coef.coef_z[4],&coef.coef_z[5]);  // sscanf to parse line file into floats then put those in arrays

            return 0; 

            printf(coef.coef_z[5]); 
        }
        else
        {
            count++;
        }
    }
    return 2; 

}





