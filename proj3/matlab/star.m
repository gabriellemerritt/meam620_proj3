%5 point star

point1 = [0 0 -1];
point2 = [0 1 -2]/1.5; 
point3 = [0 2 -1]/1.5;
point4 = [0 1.7 -2.7]/1.5;
point5 = [0 2.7 -3.7]/1.5;
point6 = [0 1.5 -3.7]/1.5;
point7 = [0 1 -4.7]/1.5;
point8 = [0 .7 -3.7]/1.5;
point9 = [0 -.4, -3.7]/1.5;
point10 = [0 .4, -2.7]/1.5;
point11 = [0 0, -1];

points = [point1; point2; point3; point4; point5; point6; point7; point8; point9; point10; point11];

plot(points(:,2), points(:,3));


%side 1
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point1.', point2.');

fileID = fopen('startraj.txt', 'w');
fprintf(fileID, '%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 2
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point2.', point3.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);


%side 3
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point3.', point4.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 4
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point4.', point5.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 5
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point5.', point6.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 6
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point6.', point7.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 7
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point7.', point8.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 8
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point8.', point9.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 9
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point9.', point10.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

%side 10
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(point10.', point11.');

fileID = fopen('startraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);
