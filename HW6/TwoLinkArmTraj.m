%% Example Title
% Trajectory planning using cubic polynomial
function [a] = TwoLinkArmTraj(theta10,dtheta10, theta1f, dtheta1f,tf, nofigure)
% Input: initial and final positions and velocities.
% Output: a vector a such that the cubic polynomial is 
% a(1) + a(2)t + a(3)t^2 + a(4)t^3;

M= [1 0 0 0;
    0 1 0 0;
    1 tf tf^2 tf^3;
    0 1 2*tf 3*tf^2];
b=[theta10; dtheta10;theta1f; dtheta1f];
a=M\b;
t=0:0.01:tf;

if nofigure==1
    return
else

figure('Name','Position (degree)');
plot(t,(a(1)+a(2)*t+ a(3)*t.^2+a(4)*t.^3)*(180/pi),'LineWidth',3);
title('Position (degree)')
grid

figure('Name','Velocity (degree/s)');
plot(t,a(2)+ 2*a(3)*t +3*a(4)*t.^2,'LineWidth',3);
title('Velocity (degree/s)')
grid

end
end



