/* Trajectory Planning for ARDrone's Bebop 
by Lou Lin and Gabby Merritt 
*/ 


#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h> 
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
#include "traj_gen.h"

#define KPX 300
#define KPY 300
#define KPZ 500

#define KDX 200
#define KDY 200
#define KDZ 00


// volatile int line_number; 


void followTrajectory(TRAJECTORY_t traj, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData;

    float ax_des, ay_des, yaw;
    // angles in percentage of degrees ? ? ? 
    yaw = deviceManager->flightStates.yaw_cur;
     //find desired acceleration in the x and y direction
     //ax_des = deviceManager->hoverTraj.ax_des + KDX * (traj.vx_des - deviceManager->flightStates.vx_cur) + KPX * (traj.x_des - deviceManager->flightStates.x_cur);
     //ay_des = deviceManager->hoverTraj.ay_des + KDY * (traj.vy_des - deviceManager->flightStates.vy_cur) + KPY * (traj.y_des - deviceManager->flightStates.y_cur);
    ax_des = traj.ax_des + KDX * (traj.vx_des - deviceManager->flightStates.vx_cur) + KPX * (traj.x_des - deviceManager->flightStates.x_cur);
    ay_des = traj.ay_des + KDY * (traj.vy_des - deviceManager->flightStates.vy_cur) + KPY * (traj.y_des - deviceManager->flightStates.y_cur);
    //put them into desired angle for the attitude controller
    deviceManager->dataPCMD.roll = -(1/9.81)*(ax_des*sin(yaw) - ay_des*cos(yaw));
    deviceManager->dataPCMD.pitch = (1/9.81)*(ax_des*cos(yaw) + ay_des*sin(yaw));
    // deviceManager->dataPCMD.gaz = -(traj.vz_des + KPZ * (traj.z_des - deviceManager->flightStates.z_cur));
    //deviceManager->dataPCMD.gaz = -(traj.vz_des + KPZ * (traj.z_des - deviceManager->flightStates.z_cur));
    deviceManager->dataPCMD.gaz = -( KDZ* (traj.vz_des - deviceManager->flightStates.vz_cur) + KPZ * (traj.z_des - deviceManager->flightStates.z_cur));
    // arent we sending 
    //print for debug
    IHM_ShowDes(deviceManager->ihm, deviceManager->hoverTraj.x_des, deviceManager->hoverTraj.y_des, ax_des, ay_des, deviceManager->dataPCMD.roll, deviceManager->dataPCMD.pitch);
}


void genTrajectory (void *customData) 
{

    float t; 
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData; 
// current time  // 
    
    t = (float)(clock() - deviceManager->genTraj.trajStartTime)/CLOCKS_PER_SEC;

// setting desired position //

	deviceManager->genTraj.x_des = deviceManager->coef.coef_x[0] + deviceManager->coef.coef_x[1]*t + deviceManager->coef.coef_x[2]*pow(t,2) + 
                 deviceManager->coef.coef_x[3]*pow(t,3)+ deviceManager->coef.coef_x[4]*pow(t,4) 
                 +deviceManager->coef.coef_x[5]*pow(t,5); 

    deviceManager->genTraj.y_des = deviceManager->coef.coef_y[0] + deviceManager->coef.coef_y[1]*t + deviceManager->coef.coef_y[2]*pow(t,2) + 
                 deviceManager->coef.coef_y[3]*pow(t,3)+ deviceManager->coef.coef_y[4]*pow(t,4) 
                 +deviceManager->coef.coef_y[5]*pow(t,5); 

    deviceManager->genTraj.z_des = deviceManager->coef.coef_z[0] + deviceManager->coef.coef_z[1]*t + deviceManager->coef.coef_z[2]*pow(t,2) + 
                 deviceManager->coef.coef_z[3]*pow(t,3)+ deviceManager->coef.coef_z[4]*pow(t,4) 
                 +deviceManager->coef.coef_z[5]*pow(t,5); 

//setting desired velocity //

	deviceManager->genTraj.vx_des =  deviceManager->coef.coef_x[1] + 2*deviceManager->coef.coef_x[2]*t + 3*deviceManager->coef.coef_x[3]*pow(t,2) +
                   4*deviceManager->coef.coef_x[4]*pow(t,3) + 5*deviceManager->coef.coef_x[5]*pow(t,4);


    deviceManager->genTraj.vy_des =  deviceManager->coef.coef_y[1] + 2*deviceManager->coef.coef_y[2]*t + 3*deviceManager->coef.coef_y[3]*pow(t,2) +
                   4*deviceManager->coef.coef_y[4]*pow(t,3) + 5*deviceManager->coef.coef_y[5]*pow(t,4);

    deviceManager->genTraj.vz_des =  deviceManager->coef.coef_z[1] + 2*deviceManager->coef.coef_z[2]*t + 3*deviceManager->coef.coef_z[3]*pow(t,2) +
                   4*deviceManager->coef.coef_z[4]*pow(t,3) + 5*deviceManager->coef.coef_z[5]*pow(t,4);

// Setting desired accelerations //

    deviceManager->genTraj.ax_des = 2*deviceManager->coef.coef_x[2] + 6*deviceManager->coef.coef_x[3]*t + 12*deviceManager->coef.coef_x[4]*pow(t,2) + 
                   20* deviceManager->coef.coef_x[5]*pow(t,3);
    
    deviceManager->genTraj.ay_des = 2*deviceManager->coef.coef_y[2] + 6*deviceManager->coef.coef_y[3]*t + 12*deviceManager->coef.coef_y[4]*pow(t,2) + 
                   20* deviceManager->coef.coef_y[5]*pow(t,3);
    
    deviceManager->genTraj.az_des = 2*deviceManager->coef.coef_z[2] + 6*deviceManager->coef.coef_z[3]*t + 12*deviceManager->coef.coef_z[4]*pow(t,2) + 
                   20* deviceManager->coef.coef_z[5]*pow(t,3);



// // Follow Traj using calculated desired values // 

//     followTrajectory(deviceManager->genTraj, deviceManager); 




}

// return positive int if succesfully read line  
// return -1  if can't open file  
// return -2  finished reading file or unknown error 
int readTrajectory (const char* file_name, int line_number, void *customData)
{ 

    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData;

	FILE *traj_file; 
	traj_file = fopen(file_name, "r"); 
	if (!traj_file)
	{
		printf("Can't Open File \n"); 
		return -1; // file error 
	}
	char line[1024]; 
	int count = 0; 
     while(fgets(line, sizeof(line), traj_file) != NULL) /* read a line */
     {
        if (count == line_number)
         {
            fgets(line, sizeof(line), traj_file); /* read a line */

            // AX0 AY0 AZ0 AX1 AY1 

            // sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
            //  &deviceManager->coef.traj_time, &deviceManager->coef.coef_x[0],&deviceManager->coef.coef_x[1], &deviceManager->coef.coef_x[2], &deviceManager->coef.coef_x[3],
            //  &deviceManager->coef.coef_x[4],&deviceManager->coef.coef_x[5], &deviceManager->coef.coef_y[0],&deviceManager->coef.coef_y[1],&deviceManager->coef.coef_y[2],
            //  &deviceManager->coef.coef_y[3],&deviceManager->coef.coef_y[4],&deviceManager->coef.coef_y[5], &deviceManager->coef.coef_z[0], &deviceManager->coef.coef_z[1],
            //  &deviceManager->coef.coef_z[2],&deviceManager->coef.coef_z[3],&deviceManager->coef.coef_z[4],&deviceManager->coef.coef_z[5]);  // sscanf to parse line file into floats then put those in arrays

            sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &deviceManager->coef.traj_time, &deviceManager->coef.coef_x[0], &deviceManager->coef.coef_y[0],
                   &deviceManager->coef.coef_z[0], &deviceManager->coef.coef_x[1], &deviceManager->coef.coef_y[1], &deviceManager->coef.coef_z[1], &deviceManager->coef.coef_x[2], 
                   &deviceManager->coef.coef_y[2], &deviceManager->coef.coef_z[2],  &deviceManager->coef.coef_x[3], &deviceManager->coef.coef_y[3], &deviceManager->coef.coef_z[3],
                   &deviceManager->coef.coef_x[4], &deviceManager->coef.coef_y[4], &deviceManager->coef.coef_z[4],  &deviceManager->coef.coef_x[5], &deviceManager->coef.coef_y[5],
                   &deviceManager->coef.coef_z[5]); 

            return line_number+1; 

            printf("last number is : %f", deviceManager->coef.traj_time); 
           }
        else
         {
             count++;
         }
     }
    return -2; 

}

void generateTrajectory(void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData;
    float t;
    float t1 = 3;
    float t2 = 6;
    static float a1 = -0.0602;
    static float b1 = 0.2917;
    static float c1 = 0;
    static float d1 = 0;
    static float a2 = 0.0509;
    static float b2 = -0.7083;
    static float c2 = 3;
    static float d2 = -3;
    t = (float)(clock() - deviceManager->hoverTraj.trajStartTime)/CLOCKS_PER_SEC;
    if(t <= t1)
    {
        deviceManager->hoverTraj.z_des = -1-(a1*pow(t,3) + b1*pow(t,2) + c1*t + d1 + deviceManager->hoverTraj.x_offset);
        deviceManager->hoverTraj.vz_des = -(3*a1*pow(t,2) + 2*b1*t + c1);
        deviceManager->hoverTraj.az_des = - (6*a1*t + 2*b1);
    }
    else if(t >= t1 && t <= t2)
    {
        deviceManager->hoverTraj.z_des = -1-(a2*pow(t,3) + b2*pow(t,2) + c2*t + d2 + deviceManager->hoverTraj.x_offset);
        deviceManager->hoverTraj.vz_des = -(3*a2*pow(t,2) + 2*b2*t + c2);
        deviceManager->hoverTraj.az_des = -(6*a2*t + 2*b2);
    }
    else
    {
        deviceManager->hoverTraj.trajStartTime = clock();
        deviceManager->Traj_on = 0;
    }
}

int lengthTrajectory(const char* file_name)
{
    FILE *traj_file; 
    traj_file = fopen(file_name, "r"); 
    if (!traj_file)
    {
        printf("Can't Open File \n"); 
        return -1; // file error 
    }
    char line[1024]; 
    int count = 0; 
    while(fgets(line, sizeof(line), traj_file) != NULL) /* read a line */
    {
        count++; 
    }
    fclose(traj_file); 

    return count; 

}
void runTrajectory(eIHM_INPUT_EVENT event, void *customData, const char* file_name)
{
// point to deviceManager // 


    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData; 
    
    clock_t t_start;
    float t_elapsed; 
    int loop_runs; 
    int i; 

// figure out how segments (from way point to way point) // 

    loop_runs = lengthTrajectory(file_name)-1; 

// move from way point to way point 
    for (i = 0; i< loop_runs; i++)
    {
        //deviceManager->genTraj.trt_start = clock(); 

        deviceManager->genTraj.trajStartTime = clock(); 

       if(readTrajectory(file_name, i, deviceManager) > 0)
        {
        
            t_elapsed = (float)(clock() - deviceManager->genTraj.trajStartTime)/CLOCKS_PER_SEC;

// while time is less than the time it takes to complete trajectory, calculate state, command roll, pitch, yaw and thrust 
            if (deviceManager->theta_flag == 1)
            {
                while (t_elapsed < deviceManager->thetaTraj.traj_time)
                {

                    genThetaTraj(deviceManager); 
                    followTrajectory(deviceManager->genTraj, deviceManager); 
                    t_elapsed = (float)(clock() - deviceManager->genTraj.trajStartTime)/CLOCKS_PER_SEC;


                }
            }
            else{

                while (t_elapsed < deviceManager->coef.traj_time ) 
                {
                    // if ( deviceManager->ihm->onInputEventCallback(IHM_INPUT_EVENT_STOP_TRAJ, deviceManager))
                    // {
                    //     deviceManager->Traj_on = 0; 
                    // }
                    genTrajectory(deviceManager); 
                    followTrajectory(deviceManager->genTraj, deviceManager);
                    t_elapsed = (float)(clock() - deviceManager->genTraj.trajStartTime)/CLOCKS_PER_SEC;

                }
            }
        }

    }
// set the trajectory flag to 0     

    deviceManager->Traj_on = 0;
    deviceManager->theta_flag = 0;        


}

void genThetaTraj (void *customData)
{

    float t; 
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData; 
// current time  // 
    float theta, omega, alpha, r, b; 
    theta = deviceManager->thetaTraj.coef_theta[0] + deviceManager->thetaTraj.coef_theta[1]*t +
            deviceManager->thetaTraj.coef_theta[2]*pow(t,2) + deviceManager->thetaTraj.coef_theta[3]*pow(t,3)+
            deviceManager->thetaTraj.coef_theta[4]*pow(t,4) + deviceManager->thetaTraj.coef_theta[5]*pow(t,5);

    omega = deviceManager->thetaTraj.coef_theta[1] + 2*deviceManager->thetaTraj.coef_theta[2]*t + 
            3*deviceManager->thetaTraj.coef_theta[3]*pow(t,2) + 4*deviceManager->thetaTraj.coef_theta[4]*pow(t,3)+
            5*deviceManager->thetaTraj.coef_theta[5]*pow(t,4);

    alpha = 2*deviceManager->thetaTraj.coef_theta[2] + 6*deviceManager->thetaTraj.coef_theta[3]*t + 
            12*deviceManager->thetaTraj.coef_theta[4]*pow(t,2) + 20*deviceManager->thetaTraj.coef_theta[5]*pow(t,3);


    t = (float)(clock() - deviceManager->genTraj.trajStartTime)/CLOCKS_PER_SEC;

    r = 2.5; 
    b = 0.25;

    deviceManager->genTraj.x_des  = r*cos(theta)-r;
    deviceManager->genTraj.y_des  = r*sin(theta);
    deviceManager->genTraj.z_des  = b*theta;

//velocities
    deviceManager->genTraj.vx_des = -r*omega*sin(theta);
    deviceManager->genTraj.vy_des = r*omega*cos(theta);
    deviceManager->genTraj.vz_des = b*omega;

// accel 

    deviceManager->genTraj.ax_des = -r*pow(omega,2)*cos(theta)+sin(theta)*(-r*alpha); 
    deviceManager->genTraj.ay_des = -r*pow(omega,2)*sin(theta)+cos(theta)*r*alpha;
    deviceManager->genTraj.az_des = b*alpha;

}
int readThetaTrajectory (const char* file_name, int line_number, void *customData)
{ 

    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*) customData;

    FILE *traj_file; 
    traj_file = fopen(file_name, "r"); 
    if (!traj_file)
    {
        printf("Can't Open File \n"); 
        return -1; // file error 
    }
    char line[1024]; 
    int count = 0; 
     while(fgets(line, sizeof(line), traj_file) != NULL) /* read a line */
     {
        if (count == line_number)
         {
            fgets(line, sizeof(line), traj_file); /* read a line */

            // AX0 AY0 AZ0 AX1 AY1 

            // sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
            //  &deviceManager->coef.traj_time, &deviceManager->coef.coef_x[0],&deviceManager->coef.coef_x[1], &deviceManager->coef.coef_x[2], &deviceManager->coef.coef_x[3],
            //  &deviceManager->coef.coef_x[4],&deviceManager->coef.coef_x[5], &deviceManager->coef.coef_y[0],&deviceManager->coef.coef_y[1],&deviceManager->coef.coef_y[2],
            //  &deviceManager->coef.coef_y[3],&deviceManager->coef.coef_y[4],&deviceManager->coef.coef_y[5], &deviceManager->coef.coef_z[0], &deviceManager->coef.coef_z[1],
            //  &deviceManager->coef.coef_z[2],&deviceManager->coef.coef_z[3],&deviceManager->coef.coef_z[4],&deviceManager->coef.coef_z[5]);  // sscanf to parse line file into floats then put those in arrays

            sscanf(line, "%f %f %f %f %f %f %f", &deviceManager->thetaTraj.traj_time,&deviceManager->thetaTraj.coef_theta[0], 
                  &deviceManager->thetaTraj.coef_theta[1], &deviceManager->thetaTraj.coef_theta[2], &deviceManager->thetaTraj.coef_theta[3],
                  &deviceManager->thetaTraj.coef_theta[4],&deviceManager->thetaTraj.coef_theta[5]); 

            return line_number+1; 

            printf("last number is : %f", deviceManager->coef.traj_time); 
           }
        else
         {
             count++;
         }
     }
    return -2; 

}





