function [] = traj_plots(filename_actual, filename_desired) 

%filename = 'example_log.txt'; 
%clear all; clf; 

 
fileID = fopen(filename_actual, 'r');
stringcells = textscan(fileID,'%s','Delimiter','\n');
stringcells = stringcells{1};
[S, ~ ] = size(stringcells); 

TIME = []; 
%ROLL = []; 
%PITCH = []; 
%YAW = []; 
X = [];
Y = [];
Z = [];
VX = [];
VY = [];
VZ = [];
for i = 1:S
    line = stringcells(i);
    line = line{1};
    %time
    Key = 'TIME: ';
    Index = strfind(line, Key); 
    time = sscanf(line(Index(1) + length(Key):end), '%f');
    TIME = [TIME; time];
%     %roll
%     Key = 'ROLL: ';
%     Index = strfind(line, Key); 
%     roll = sscanf(line(Index(1) + length(Key):end), '%f');
%     ROLL = [ROLL; roll]; 
%     %pitch
%     Key = 'PITCH: ';
%     Index = strfind(line, Key); 
%     pitch = sscanf(line(Index(1) + length(Key):end), '%f');
%     PITCH = [PITCH; pitch]; 
%     %yaw
%     Key = 'YAW: ';
%     Index = strfind(line, Key); 
%     yaw = sscanf(line(Index(1) + length(Key):end), '%f');
%     YAW = [YAW; yaw]; 
    %X
    Key = 'X: ';
    Index = strfind(line, Key); 
    x = sscanf(line(Index(1) + length(Key):end), '%f');
    X = [X; x];
    %Y
    Key = 'Y: ';
    Index = strfind(line, Key); 
    y = sscanf(line(Index(1) + length(Key):end), '%f');
    Y = [Y; y];
    %Z
    Key = 'ALTITUDE: ';
    Index = strfind(line, Key); 
    z = sscanf(line(Index(1) + length(Key):end), '%f');
    Z = [Z; z];
     %VX
    Key = 'VX: ';
    Index = strfind(line, Key); 
    vx = sscanf(line(Index(1) + length(Key):end), '%f');
    VX = [VX; vx];
    %Y
    Key = 'VY: ';
    Index = strfind(line, Key); 
    vy = sscanf(line(Index(1) + length(Key):end), '%f');
    VY = [VY; vy];
    %Z
    Key = 'VZ: ';
    Index = strfind(line, Key); 
    vz = sscanf(line(Index(1) + length(Key):end), '%f');
    VZ = [VZ; vz];
    
end

fileID = fopen(filename_desired, 'r');
stringcells = textscan(fileID,'%s','Delimiter','\n');
stringcells = stringcells{1};
[S, ~ ] = size(stringcells); 

TIMEDES = []; 
XDES = [];
YDES = [];
ZDES = [];
VXDES = [];
VYDES = [];
VZDES = [];
for i = 1:S
    line = stringcells(i);
    line = line{1};
    %time
    Key = 'TIME: ';
    Index = strfind(line, Key); 
    time = sscanf(line(Index(1) + length(Key):end), '%f');
    TIMEDES = [TIMEDES; time];
    %X
    Key = 'XDES: ';
    Index = strfind(line, Key); 
    x = sscanf(line(Index(1) + length(Key):end), '%f');
    XDES = [XDES; x];
    %Y
    Key = 'YDES: ';
    Index = strfind(line, Key); 
    y = sscanf(line(Index(1) + length(Key):end), '%f');
    YDES = [YDES; y];
    %Z
    Key = 'ZDES: ';
    Index = strfind(line, Key); 
    z = sscanf(line(Index(1) + length(Key):end), '%f');
    ZDES = [ZDES; z];
     %VX
    Key = 'VXDES: ';
    Index = strfind(line, Key); 
    vx = sscanf(line(Index(1) + length(Key):end), '%f');
    VXDES = [VXDES; vx];
    %Y
    Key = 'VYDES: ';
    Index = strfind(line, Key); 
    vy = sscanf(line(Index(1) + length(Key):end), '%f');
    VYDES = [VYDES; vy];
    %Z
    Key = 'VZDES: ';
    Index = strfind(line, Key); 
    vz = sscanf(line(Index(1) + length(Key):end), '%f');
    VZDES = [VZDES; vz];
    
end

figure(1); 
subplot(3,1,1)
plot(TIME, X, 'b' , TIMEDES, XDES, 'r')
xlabel('time (s)');
ylabel('X (m)');
subplot(3,1,2)
plot(TIME, Y, 'b' , TIMEDES, YDES, 'r')
xlabel('time (s)');
ylabel('Y (m)');
subplot(3,1,3);
plot(TIME, Z, 'b' , TIMEDES, ZDES, 'r')
xlabel('time (s)');
ylabel('Z (m)');
title('Desired vs. Actual Position')

figure(2);
subplot(3,1,1)
plot(TIME, VX, 'b' , TIMEDES, VXDES, 'r')
xlabel('time (s)');
ylabel('X Velocity (m/s)');
subplot(3,1,2)
plot(TIME, VY, 'b' , TIMEDES, VYDES, 'r')
xlabel('time (s)');
ylabel('Y Velocity (m/s)');
subplot(3,1,3);
plot(TIME, VZ, 'b' , TIMEDES, VZDES, 'r')
xlabel('time (s)');
ylabel('Z Velocity (m)');
title('Desired vs. Actual Velocity');

figure(3); 
plot3(XDES,YDES,ZDES,'r')
hold on 
plot3(X,Y,Z,'b');

end

