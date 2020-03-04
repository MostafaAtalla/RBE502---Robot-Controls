%Steering control
T=50;
x0 =[0;0;0.1]; % Feel free to change the initial state and sampling horizon.

% Some parameters for the system. 
a=1;
b=2;



%K is additional control gain to pass to the system
%% homework 1: design steering control to follow a straight lane.
%TODO: param is the additional parameter to pass to the ode function.
% k1=0.5;
% k2=0.5;
% param= [k1 k2];
%[T,X] = ode45(@(t,x) ode_bicycle(t,x, param), [0:T], x0, param);

%% homework 2: design steering control to follow a straight lane while maintaining a given velocity
% x0 = [0;5.1;0.05];
% k1 = 3.5;
% k2 = 3.5;
% L = 20;
% param= [k1 k2 L];
% %TODO: param is the additional parameter to pass to the ode function.
% [T,X] = ode45(@(t,x) ode_dubins(t,x, param), [0:T], x0, param);

% plot your state trajectories for both 1 and 2, using the following code or else.
figure
plot(T,X(:,1),'LineWidth',4);
xlabel('t');
ylabel('x');

figure
plot(T,X(:,2),'LineWidth',4);
xlabel('t');
ylabel('y');

figure
plot(T,X(:,3),'LineWidth',4);
xlabel('t');
ylabel('theta');

figure
plot(X(:,1), X(:,2))


