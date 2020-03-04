%This code has been developed by: Mostafa Atalla%
%HW:4 for RBE502 Robotics Control Course by: Prof. Jie Fu%

clc
clear all;
close all;

%% Initialization %%
x0=[40 40 0.3 0.1];          %Setting initial conditions for the state vector
xf = [10,20, 0, 0];            %Final state desired
kp=[250 0;0 250];            %Proportional gain vector
kd=[160 0;0 160];              %Dervative gain vector
tf=15;                       %Final time 

%% Implement the PD control for set point tracking %%
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) odefcn(t,x,xf,kp,kd),[0 tf],x0, options);

%% Plotting theta1,theta2 under the PD controller %%
figure('Name', 'Theta1, Theta2 under PD SetPoint Control')
movegui(1,'northeast')
plot(T, (X(:,1)*(180/pi)),'r-');     
hold on
plot(T, (X(:,2)*(180/pi)),'r--');
hold on
title('Theta1, Theta2 under PD SetPoint Control');
xlabel('TIME') 
ylabel('THETA (Degrees)')
legend('Theta1','Theta2')
grid on



%% Animation of the TWO Link Manipulator %%

%Manipulator Parameters%
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1; 

%Initialization of Tip point of every Link%
[k j]=size(X);
pointl1=zeros(k,2); %End Point of Link1
pointl2=zeros(k,2); %End Point of Link2

%Create Figure for the Animation%
figure('Name', 'Animation of the 2 Link Manpiulator under PD Control')
movegui(2,'northwest')

