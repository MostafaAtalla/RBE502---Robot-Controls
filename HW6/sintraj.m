%% Example Title
% Trajectory planning using cubic polynomial
function [a] = sintraj(tf, nofigure)
% Input: initial and final positions and velocities.
% Output: a vector a such that the cubic polynomial is 
% a(1) + a(2)t + a(3)t^2 + a(4)t^3;


t=0:0.01:tf;
if nofigure==1
    return
else

figure('Name','Position (degree)');
plot(t,-sin(t),'LineWidth',3);
title('Position (degree)')
grid

figure('Name','Velocity (degree/s)');
plot(t,-cos(t),'LineWidth',3);
title('Velocity (degree/s)')
grid

end
end



