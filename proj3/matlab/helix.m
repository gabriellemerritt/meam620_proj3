
%total time 
total_time = 12; %sec
theta_max = 2*pi;
t0 = 0;
tf = 12;

solutions = [0; theta_max;0;0;0;0];
equations = [1 t0 t0^2 t0^3 t0^4 t0^5;
            1 tf tf^2 tf^3 tf^4 tf^5;
            0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 6*t0 12*t0^2 20*t0^3;
            0 0 2 6*tf 12*tf^2 20*tf^3];
coefficients = equations\solutions; %coefficients = [a0;a1;a2;a3;a4;a5]
a0 = coefficients(1);
a1 = coefficients(2);
a2 = coefficients(3);
a3 = coefficients(4);
a4 = coefficients(5);
a5 = coefficients(6);

%Use these equations for the coefficients

% %fill in the thetas matrix column by column with the time-based
% %position
% theta = a0 + a1*t +a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
% omega = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
% alpha = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
% 
% %position
% 
 r = 5; %m, radius
 z_max = -2.5; %m
% b=z_max/(theta_max);
% 
% x = r*cos(theta)-r;
% y = r*sin(theta);
% z = b*theta;
% 
% %velocities
% x_vel = -r*omega*sin(theta);
% y_vel = r*omega*cos(theta);
% z_vel = b*omega;
% 
% %accelerations 
% x_accel = -r*omega^2*cos(theta)+sin(theta)*(-r*alpha); 
% y_accel = -r*omega^2*sin(theta)+cos(theta)*r*alpha;
% z_accel = b*alpha;

fileID = fopen('helixtraj.txt', 'w');
fprintf(fileID, '\n%0.9f',  total_time);
fprintf(fileID,'%0.9f ',a0,a1,a2,a3,a4,a5);
fclose(fileID);

%back to starting position
a=[0,0,-z_max].';
b=[0,0,-1].';
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(a, b);

fileID = fopen('helixtraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

