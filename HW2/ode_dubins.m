function dz = ode_dubins(t,z,param)

%% The following code for steering control to follow straight lane with velocity scheduling.
vr = 2;
yr=5;
b = 2;
k1=param(1);
k2=param(2);
L=param(3);

dz =zeros(3,1);

%% TODO: Here is the code for control input.
%%% the controller needs to provide control input: linear velocity and
%%% steering angle: v, delta
 % Calculate the current velocity V0 and the steering angle delta
    v0= -L * (z(1) - vr *t) + vr;
    delta = (k1 *b * (yr - z(2))) / vr - ((k2 * vr * z(3)) / b);

theta= z(3);

dz(1) = v0*cos(theta);
dz(2) = v0*sin(theta);
dz(3) = (v0/b)*tan(delta);
end