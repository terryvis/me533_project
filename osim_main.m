% simulation code for me533 final project/term paper
% simulate musculoskeletal geometry model (2nd-order poly)
% using OpenSim arm26 model

clc;close all;clear variables;

% import opensim packages
import org.opensim.modeling.*;

% initialize model
model = Model('arm26.osim');
state = model.initSystem();

% setup simulation
N = 250;
tf = 10;
t = linspace(0,tf,N);
dt = tf/(N-1);

% define x and u as time-series (vectors)
% u = phi (joint angle)
u = zeros(N,1);
u(1) = model.getJointSet().get(1).getCoordinate().getValue(state);

% x = l_0 (fiber length 0 - triceps long)
% also define x_dot vector
x = zeros(N,1);
x_dot = zeros(N,1);

% dimension of input and output
num_controls = 1;
num_st = 1;

% get initial state x0
x(1) = model.getMuscles().get(3).getFiberLength(state);
fprintf('x0 = %f\n', x(1));

% define desired trajectory and deriv - can change
% x_d = -0.01*sin(pi*t) + 0.15;
x_d = 0.14*ones(1,N);
x_ddot = [0 (x_d(2:end)-x_d(1:end-1))/dt];

% define param vector
ord = 2;
% a = 0.15*ones(ord+1,1); % [a0;a1;a2]
a = [0.15 -0.1 0.1];

% define additional params for control/adaptation
lambda = 1/(dt*10);
P = diag([1,0,1,1]);

% run simulation
for i = 2:N
    % advance timestep
    state.setTime(t(i));

    % calculate phi (u(t)) based on adaptive control policy
    
    % poly coefficients
    a0 = a(1);
    a1 = a(2);
    a2 = a(3);

    % define u, u_dot for states (Y) based on opensim joint
    % position/velocity
    if i >= 3
        u_dot = (u(i-1) - u(i-2))/dt;
    else
        u_dot = 0;
    end
    u_prev = model.getJointSet().get(1).getCoordinate().getValue(state);

    % define Y (states)
    Y = [1 u_dot (lambda*u_prev^2 + 2*u_prev*u_dot) (x_ddot(i-1) + lambda*x_d(i-1))];
    
    % define controller params
    a_c = [-a0/a1; -1/lambda; -a2/(lambda*a1); 1/(lambda*a1)];
    k = -1/(lambda*a1);

    % define s (x_dot - x_ddot + lambda*x - lambda-x_d)
    s = x_dot(i-1) - x_ddot(i-1) + lambda*(x(i-1) - x_d(i-1));

    % compute input (angle)
    u(i) = Y*a_c - k*s;
    % u(i) = pi/2 + pi/2*sin(t(i));

    % update controller params based on adaptation law
    % a_c = a_c - dt*P*Y'*s;

    % compute a0, a1, a2
    a1_new = 1/(lambda*a_c(4));
    a0_new = -a_c(1)*a1_new;
    a2_new = -a_c(3)*a1_new;
    a = [a0_new;a1_new;a2_new];

    % set joint angle according to above policy
    model.getJointSet().get(1).getCoordinate().setValue(state,u(i));

    % let muscle come to equilibrium
    model.equilibrateMuscles(state);
    
    % compute x and x_dot
    x(i) = model.getMuscles().get(3).getFiberLength(state);
    x_dot(i) = model.getMuscles().get(3).getFiberVelocity(state);

    disp(model.getMuscles().get(3).getActivation(state));
end


% plot results
subplot(2,1,1),plot(t,u)
subplot(2,1,1),xlabel('Time [sec]')
subplot(2,1,1),ylabel('Joint angle [rad]')

subplot(2,1,2),plot(t,x)
subplot(2,1,2),hold on
subplot(2,1,2),plot(t,x_d)
subplot(2,1,2),xlabel('Time [sec]')
subplot(2,1,2),ylabel('Normalized fiber length')
subplot(2,1,2),legend('actual','desired')

fprintf('rmse = %f\n', mean((x - x_d').^2))