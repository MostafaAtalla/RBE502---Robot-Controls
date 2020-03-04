%% HW6 Code for implementing 5 different controllers on Two Link Manipulator
%% By: Mostafa Atalla
%% RBE 502 Robotics Control course by Prof. Jie Fu

clc
clear all;
close all;

global o

%% Initialization for Trajectory Tracking%% 
% Setting the initial,final conditions and the time span.
x0=[1 3];       %Setting initial conditions for the state vector
x0_traj=[0.5 1];    %Initial condition for the trajectory
xf = [2,1];     %Final state desired
tf=20;            %Final time 
step=0.01         %time step

tspan=0:step:tf;  %time span vector

%% Adaptive Passivity based Controller Input
lambda=15;         %Lambda Square positive definite matrix for lyapanuv Based Controller.
kv=15;             %Kv matrix.
L=[20 0 0;0 20 0;0 0 20];              %Symmetric Positive definite matrix

cont_input={lambda,kv,L}    %Controller input cell

%original system paramters%
I=7.5;
mgd=6;
fv=1.5;

original_param=[I mgd fv]; %original parameters vector

%Initial estimate%
I_e=8;
mgd_e=5;
fv_e=2.5;

estimate_param=[I_e mgd_e fv_e]; %estimated parameters vector

% Initial alfa
alfa0=[mgd_e; fv_e; I_e]

%% Inverse Dynamics Robust Controller
gama=[1 1 1 1];
p=[1 0;0 1];
B=[0;1];
kp=1;
kd=1;

%% GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.
nofigure=2;
[a] = TwoLinkArmTraj(x0_traj(1),x0_traj(2), xf(1),xf(2),tf, nofigure) %Trajectory Generation for the first Link

ddtheta=0;
%% Implement the inverse dynamic control  
%options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4]);
%[T,X] = ode45(@(t,x) adaptivePassive(t,x,a,cont_input,original_param,estimate_param,step,alfa0,ddtheta),[0 tf],x0, options)

%figure(1);
%hold on
%plot(T, X(:,1)*(180/pi),'m-');
%legend('Position Trajectory','Passivity Based Adaptive Controller')
%title('Position Convergence Under Passivity Based Adaptive Controller')
%xlabel('Time')
%ylabel('Theta (Degrees)')

%figure(2);
%hold on
%plot(T, X(:,2),'m--');
%legend('Velocity Trajectory','Passivity Based Adaptive Controller')
%title('Velociy Convergence Under Passivity Based Adaptive Controller')
%xlabel('Time')
%ylabel('Theta_dot')

%% Inverse Dynamics Robust Controller 
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4]);
[T,Y] = ode45(@(t,y) inverseDCRobust(t,y,a,gama,B,p,kp,kd,estimate_param,original_param,o),[0 tf],x0, options)

figure(1);
hold on
plot(T, Y(:,1)*(180/pi),'m-');
legend('Position Trajectory','Passivity Based Adaptive Controller')
title('Position Convergence Under Passivity Based Adaptive Controller')
xlabel('Time')
ylabel('Theta (Degrees)')

figure(2);
hold on
plot(T, Y(:,2),'m--');
legend('Velocity Trajectory','Passivity Based Adaptive Controller')
title('Velociy Convergence Under Passivity Based Adaptive Controller')
xlabel('Time')
ylabel('Theta_dot')