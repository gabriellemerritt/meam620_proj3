
%side 1
a=[0,0,-1].';
b=[0,0,-2].'; 
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(a, b);
POS = []; 
for t = 0:.1:2
    pos = a0.'+a1.'*t+a2.'*t^2+a3.'*t^3+a4.'*t^4+a5.'*t^5;
    POS = [POS pos];
end

fileID = fopen('squaretraj.txt', 'w');
fprintf(fileID, '\n'); 
fprintf(fileID, '%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

a=[0,0,-2].';
b=[1,0,-2].';
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(a, b);
 
for t = 0:.1:2
    pos = a0.'+a1.'*t+a2.'*t^2+a3.'*t^3+a4.'*t^4+a5.'*t^5;
    POS = [POS pos];
end
    

%side 2
fileID = fopen('squaretraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);
for t = 0:.1:2
    pos = a0.'+a1.'*t+a2.'*t^2+a3.'*t^3+a4.'*t^4+a5.'*t^5;
    POS = [POS pos];
end


%side 3
a=[1,0,-2].';
b=[1,0,-1].';
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(a, b);
for t = 0:.1:2
    pos = a0.'+a1.'*t+a2.'*t^2+a3.'*t^3+a4.'*t^4+a5.'*t^5;
    POS = [POS pos];
end

fileID = fopen('squaretraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);
for t = 0:.1:2
    pos = a0.'+a1.'*t+a2.'*t^2+a3.'*t^3+a4.'*t^4+a5.'*t^5;
    POS = [POS pos];
end

%side 4
a=[1,0,-1].';
b=[0,0,-1].';
[total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(a, b);
for t = 0:.1:2
    pos = a0.'+a1.'*t+a2.'*t^2+a3.'*t^3+a4.'*t^4+a5.'*t^5;
    POS = [POS pos];
end

fileID = fopen('squaretraj.txt', 'a+');
fprintf(fileID, '\n%0.9f ',  total_time);
fprintf(fileID,'%0.9f ',a0, a1, a2, a3, a4, a5);
fclose(fileID);

plot3(POS(1,:), POS(2,:), POS(3,:))



 