% simulation code for me533 final project/term paper
% simulate % simulate joint dynamics model (2nd order mass/spring/damper)
% using plant data from predefined I/B/K model

clc;close all;clear variables;

% setup simulation
N = 1000;
tf = 10;
t = linspace(0,tf,N);
dt = tf/(N-1);

% define x and u as time-series (vectors)
% u = torque
% initialize torque to zero
u = zeros(N,1);

% x = phi (joint angle)
% also define x_dot vector
x = zeros(N,1);
x_dot = zeros(N,1);

% dimension of input and output
num_controls = 1;
num_st = 1;

% define initial state x0
x(1) = 0.85; % same as with opensim
fprintf('x0 = %f rad\n', x(1));

% define desired trajectory and deriv - can change
x_d = pi/3*ones(1,N);% + sin(pi*t);
x_ddot = [0 (x_d(2:end)-x_d(1:end-1))/dt];
x_d_accel = [0 (x_ddot(2:end)-x_ddot(1:end-1))/dt];

% define param vector a = [I B K]' with initial guesses
a = 0.5*ones(3,1);

% define additional params for control/adaptation
lambda = 1/(dt*10);
k = 1;
P = 0.01*eye(3);

% run simulation
for i = 2:N
    % calculate torque (u(t)) based on adaptive control policy
    
    % get params
    I = a(1);
    B = a(2);
    K = a(3);

    % define Y (states)
    Y = [(lambda * (x_ddot(i-1) - x_d(i-1)) + x_d_accel(i-1)) x_dot(i-1) x(i-1)];

    % define s (x_dot - x_ddot + lambda*x - lambda-x_d)
    s = x_dot(i-1) - x_ddot(i-1) + lambda*(x(i-1) - x_d(i-1));

    % compute input (angle)
    u(i) = Y*a - k*s;

    % update controller params based on adaptation law
    a = a - dt*P*Y'*s;

    % TODO: use ode solver to compute phi based on torque
    
    % define y = [x, x_dot]
    odefun = @(t,y) [y(2); (u(i) - B*y(2) - K*y(1))/I];

    y0 = [x(i-1), x_dot(i-1)];
    tspan = t(i-1:i);
    [t_ode,y] = ode45(odefun, tspan, y0);

    % compute x and x_dot
    x(i) = y(1);
    x_dot(i) = y(2);

    % debug - print states
    fprintf('Time: %f, Applied Torque: %f, Angle: %f\n', t(i), u(i), x(i));

end


% plot results
subplot(2,1,1),plot(t,u)
subplot(2,1,1),xlabel('Time [sec]')
subplot(2,1,1),ylabel('Applied torque [Nm]')

subplot(2,1,2),plot(t,x)
subplot(2,1,2),hold on
subplot(2,1,2),plot(t,x_d)
subplot(2,1,2),xlabel('Time [sec]')
subplot(2,1,2),ylabel('Joint angle [rad]')
subplot(2,1,2),legend('actual','desired')

fprintf('rmse = %f\n', sqrt(mean((x - x_d').^2)))