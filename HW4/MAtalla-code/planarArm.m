%% Task Space PD+Gravity Compensation Controller
%% RBE 502 Robotics Control by Prof. Jie Fu
%% Code Developed by: Mostafa Atalla

clc
clear all;
close all;

global H 
global J
%% Initialization %%
x0=[0 pi/2 0.3 0.1];           %Initial state of the system in angles (Assuming that we can measure encoder values)
xf = [2,0, 0, 0];              %Final state desired in cartesian coordinates
tf=20;                         %time span 

%% Control Input
kp=[400 0;0 300];
kd=[400 0;0 300];

%% Link Properties

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1; 
g=9.8;
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;




%% Implement the PD control for set point tracking %%
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) odefcn(t,x,xf,kp,kd),[0 tf],x0, options);

%% Plotting theta1,theta2 under the PD controller %%

j=size(X,1)
x_sys=zeros(1,j)
y_sys=zeros(1,j)
for i=1:j
    A=double(subs(H,[sym('theta1') sym('theta2')],[X(i,1) X(i,2)]));
    x_sys(1,i)=A(1,1)
    y_sys(1,i)=A(2,1)
end
x_sys=x_sys'
y_sys=y_sys'
figure('Name', 'X,Y Convergence under PD SetPoint Control')
movegui(1,'northeast')
plot(T, x_sys(:,1),'r-');     
hold on
plot(T, y_sys(:,1),'r--');
hold on
title('X,Y Convergence under PD SetPoint Control');
xlabel('TIME') 
ylabel('X,Y Values')
legend('X','Y')
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

%Computing the Tip Point positions corresponding to theta1, theta2%
for ii=1:k;

%Adjusting Figure for every iteration%
clf
xlabel('X - POSITION') 
ylabel('Y - POSITION') 
grid on
title('TWO LINK MANIPULATOR ANIMATION UNDER PD CONTROL');
axis([floor(min(pointl2(:,1))) 2.2 -2.2 2.2]);
axis square

%Computing the points%
pointl1(ii,1) =  l1*cos(X(ii,1)) ;     %Computing x positions of end point of link 1
pointl1(ii,2) = l1*sin(X(ii,1));       %Computing y positions of end point of link 1
pointl2(ii,1) = pointl1(ii,1) + (l2*cos(X(ii,1)+X(ii,2))); %Computing x positions of end point of link 2
pointl2(ii,2) = pointl1(ii,2)+(l2*sin(X(ii,1)+X(ii,2)));   %Computing y positions of end point of link 2

%Plotting the links%
line([0,pointl1(ii,1)],[0,pointl1(ii,2)],'linewidth',2,'color','black');
line([pointl1(ii,1),pointl2(ii,1)],[pointl1(ii,2),pointl2(ii,2)],'linewidth',2,'color','blue');

hold on

plot(0,0,'o','markersize',7)
plot(pointl1(ii,1),pointl1(ii,2),'o','markersize',7)
plot(pointl2(ii,1),pointl2(ii,2),'o','markersize',7)
plot(pointl2(:,1),pointl2(:,2),'-')
pause(.07) 



end



   



