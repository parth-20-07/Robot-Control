clc;
clear all;
close all;
%% System Information

initial_conditions = [-98;20;-100;25]
tspan = 10
%% Writing System

syms x1(t) x2(t) x3(t) x4(t) u1(t) u2(t) t 'real'
x = [x1;x2;x3;x4];
u = [u1;u2];
dx = [x2; u1; x4 ; u2/2]
%% Writing System Dynamics

A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0]
B = [0 0; 1 0; 0 0; 0 0.5]
C = [1 0 0 0; -1 0 1 0]
D = [0 0;0 0]
%% Checking Controllability

CO = ctrb(A,B)
r_CO = rank(CO)
if(rank(A) == r_CO)
    disp("System is controllable")
else
    disp("System is not controllable")
end
%% Designing LQR Controller with provided Q,R

Q = diag([2,5,2,5])
R = diag([1,1])
[P,K,L] = icare(A,B,Q,R)
u1 = - K(1,:)*x;
u2 = - K(2,:)*x;
dx = [x2; u1; x4 ; u2/2]
%% Testing System Output

vars = [x1(t) x2(t) x3(t) x4(t)];
func_dx = odeFunction(dx,vars);
fdx = @(t,x)func_dx(t,x);
[t,x] = ode45(fdx,[0,tspan],initial_conditions);
for i=1:1:size(t)
    y(i,:) = (C*x(i,:)')';
    inputs(i,:) = (-K*x(i,:)')';
end
%% Plotting Graphs

disp("Plotting States")
plot(t,x(:,1),'r')
hold on;
plot(t,x(:,3),'b')
hold on;
legend ('x_{1}','x_{3}');
grid on;
xlabel("t_{sec}")
ylabel("s_{m}")
title("Displacement vs Time")
hold off;
exportgraphics(gcf,'default_tuned_x1x3.png','Resolution',1200)

plot(t,x(:,2),'r')
hold on;
plot(t,x(:,4),'b')
hold on;
legend ('x_{2}','x_{4}');
grid on;
xlabel("t_{sec}")
ylabel("v_{m/s}")
title("Velocity vs Time")
hold off;
exportgraphics(gcf,'default_tuned_x2x4.png','Resolution',1200)

disp("Plotting Outputs")
plot(t,y(:,1),'r')
legend ('y_{1}');
grid on;
xlabel("t_{sec}")
ylabel("s_{m}")
title("Car 1 Displacement vs Time")
hold off;
exportgraphics(gcf,'default_tuned_y1.png','Resolution',1200)

plot(t,y(:,2),'r')
legend ('y_{2} = x_{3} - x_{1}');
grid on;
xlabel("t_{sec}")
ylabel("s_{m}")
title("Relative Displacement vs Time")
hold off;
exportgraphics(gcf,'default_tuned_y2.png','Resolution',1200)

disp("Plotting Inputs")
plot(t,inputs(:,1),'r')
hold on;
plot(t,inputs(:,2),'b')
hold on;
legend ('u_{1}','u_{2}');
grid on;
xlabel("t_{sec}")
ylabel("U_{force}")
title("Inputs vs Time")
hold off;
exportgraphics(gcf,'default_tuned_u1u2.png','Resolution',1200)
%% Safety Constraints Check

first_violation_time = tspan+1;
for i=1:1:size(t)
    if(y(i,2)>0)
        if(first_violation_time > tspan)
            first_violation_time = t(i)
            [maximum_violation,i] = max(y(:,2));
            maximum_violation
            max_violation_time = t(i)
            break;
        end
    end
end
%% Tuning LQR for minimum distance Tuning

a = 0.0
b = 1
first_entry = 1;
warning('off','all')
output = [0 0];
while(min(output(:,2)) < -2 || max(output(:,2))>0 || first_entry)
    first_entry=0;
    syms x1(t) x2(t) x3(t) x4(t) 'real';
    x = [x1;x2;x3;x4];
    a = a+0.05;
    Q = diag([2.5, 10, a, b-a]);
    [P,K,L] = icare(A,B,Q,R);
    u1 = - K(1,:)*x;
    u2 = - K(2,:)*x;
    dx = [x2; u1; x4 ; u2/2];
    vars = [x1(t) x2(t) x3(t) x4(t)];
    func_dx = odeFunction(dx,vars);
    fdx = @(t,x)func_dx(t,x);
    [t,x] = ode45(fdx,[0,tspan],initial_conditions);
    length = size(t);
    length = length(1,1);
    output = zeros(length,2);
    for i=1:1:size(t)
        output(i,:) = (C*x(i,:)')';
    end
    maximum = max(output(:,2));
    minimum = min(output(:,2));
    plot(t,output(:,2),'black')
    hold on;
    grid on;
    if(maximum > 0)
        disp('Failure at')
        b
        b = b+1;
        a = 0;
    end
end
exportgraphics(gcf,'tuning_performance.png','Resolution',1600)
hold off;
warning('on','all')
disp('Congratulation. System is tuned !!')
a
b
%% Displaying Performance on new Tuning

syms x1(t) x2(t) x3(t) x4(t) 'real';
x = [x1;x2;x3;x4];
Q = diag([2.5, 10, a, b-a])
[P,K,L] = icare(A,B,Q,R)
u1 = - K(1,:)*x;
u2 = - K(2,:)*x;
dx = [x2; u1; x4 ; u2/2];
vars = [x1(t) x2(t) x3(t) x4(t)];
func_dx = odeFunction(dx,vars);
fdx = @(t,x)func_dx(t,x);
[t,x] = ode45(fdx,[0,tspan],initial_conditions);
length = size(t);
length = length(1,1);
output = zeros(length,2);
for i=1:1:size(t)
    output(i,:) = (C*x(i,:)')';
    inputs(i,:) = (-K*x(i,:)')';
end
%% Plotting Graphs

disp("Plotting States")
plot(t,x(:,1),'r')
hold on;
plot(t,x(:,3),'b')
hold on;
legend ('x_{1}','x_{3}');
grid on;
xlabel("t_{sec}")
ylabel("s_{m}")
title("Displacement vs Time")
hold off;
exportgraphics(gcf,'custom_tuned_x1x3.png','Resolution',1200)

plot(t,x(:,2),'r')
hold on;
plot(t,x(:,4),'b')
hold on;
legend ('x_{2}','x_{4}');
grid on;
xlabel("t_{sec}")
ylabel("v_{m/s}")
title("Velocity vs Time")
hold off;
exportgraphics(gcf,'custom_tuned_x2x4.png','Resolution',1200)

disp("Plotting Outputs")
plot(t,output(:,1),'r')
legend ('y_{1}');
grid on;
xlabel("t_{sec}")
ylabel("s_{m}")
title("Car 1 Displacement vs Time")
hold off;
exportgraphics(gcf,'custom_tuned_y1.png','Resolution',1200)

plot(t,output(:,2),'r')
legend ('y_{2} = x_{3} - x_{1}');
grid on;
xlabel("t_{sec}")
ylabel("s_{m}")
title("Relative Displacement vs Time")
hold off;
exportgraphics(gcf,'custom_tuned_y2.png','Resolution',1200)

disp("Plotting Inputs")
plot(t,inputs(:,1),'r')
hold on;
plot(t,inputs(:,2),'b')
hold on;
legend ('u_{1}','u_{2}');
grid on;
xlabel("t_{sec}")
ylabel("U_{force}")
title("Inputs vs Time")
hold off;
exportgraphics(gcf,'custom_tuned_u1u2.png','Resolution',1200)