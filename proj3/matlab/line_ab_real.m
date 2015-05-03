function [total_time, a0, a1, a2, a3, a4, a5] = line_ab_real(a, b)
%function [desired_state] = line_ab(t, qn)

%line starts at a and goes to b
%a = [0;0;1];
%b = [1;1;2];


%total time
distance = sqrt(sum((b-a).^2));
total_time = 10*sqrt(distance); 

t0 = 0;
tf = total_time;

i=1; 
solutions = [a.'; b.';[0 0 0];[0 0 0];[0 0 0];[0 0 0]];
equations = [1 t0 t0^2 t0^3 t0^4 t0^5;
            1 tf tf^2 tf^3 tf^4 tf^5;
            0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 6*t0 12*t0^2 20*t0^3;
            0 0 2 6*tf 12*tf^2 20*tf^3];
coefficients = equations\solutions; %coefficients = [a0;a1;a2;a3;a4;a5]
a0 = coefficients(1,:);
a1 = coefficients(2,:);
a2 = coefficients(3,:);
a3 = coefficients(4,:);
a4 = coefficients(5,:);
a5 = coefficients(6,:);

end

