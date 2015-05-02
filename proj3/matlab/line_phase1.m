a=[0,0,1].';
b=[1,2,2].'; 
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(a, b);

fileID = fopen('linetraj.txt', 'w');
fprintf(fileID, '%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);
