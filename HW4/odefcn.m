%This code has been developed by: Mostafa Atalla%
%HW:4 for RBE502 Robotics Control Course by: Prof. Jie Fu%
%% Defining the 1st order ode to be solved by the ode45%%
function dxdt=odefcn(t,x,xf,kp,kd)

%Manipulator Parameters%

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1; 
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

%Inertia and Coriolos Materices Computation%

M = [a+2*b*cos(x(2)), d+b*cos(x(2)); d+b*cos(x(2)), d] %Inertia Matrix
C = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0] %Coriolos Materix
invM = inv(M);
invMC= inv(M)*C;

%Setting PD Controller for each Joint input Torque - Set Point Tracking%
%General PD Controller u=-Kp(e)-Kd*e_dot

e=[x(1)-xf(1);x(2)-xf(2)];         %position error vector
e_dot=[x(3)-xf(3);x(4)-xf(4)];     %Velocity error vector
u=-kp*e-kd*e_dot;                  %Controller


%State Space Representation%
%4 first order ode%

dxdt=zeros(4,1);   %initialization of first order ode vector%
dxdt(1)=x(3);
dxdt(2)=x(4);
dxdt(3)= invMC(1,:)*[x(3);x(4)]+invM(1,:)*u;
dxdt(4)= invMC(2,:)*[x(3);x(4)]+invM(2,:)*u;
end