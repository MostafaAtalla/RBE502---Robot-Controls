
function dz = ode_bicycle(t,z,param)
b=2;
a=1;


% The following code for steering control to follow straight lane.
v0=1; % fixed velocity 

dz =zeros(3,1);

% TODO: here insert your code for controller
%%% the controller needs to provide control input delta.
k = [param(1) param(2)]; %the calculated gains K1,K2 in vector K
delta = -k * z(2:3);
theta=z(3);
% use the input delta, evaluate the alpha.
alpha=atan((a/b)*tan(delta)); 
dz(1) = v0*cos(alpha+theta)/cos(alpha);
dz(2) = v0*sin(alpha+theta)/cos(alpha);
dz(3) = (v0/b)*tan(delta);
end