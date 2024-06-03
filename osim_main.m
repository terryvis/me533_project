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
N = 100;
tf = 5;
t = linspace(0,tf,N);
dt = (N-1)/tf;

% define x and u as time-series (vectors)
% u = phi (joint angle)
u = zeros(N,1);
u(1) = model.getJointSet().get(1).getCoordinate().getValue(state);

% x = l_0 (fiber length 0 - triceps long)
x = zeros(N,1);

% dimension of input and output
num_controls = 1;
num_st = 1;

% get initial state x0
x(1) = model.getMuscles().get(0).getFiberLength(state);
fprintf('x0 = %f\n', x(1));

% define desired trajectory - can change
x_d = 0.1*sin(pi*t) + 0.5;

% define param vector
ord = 2;
a = 0.1*ones(ord+1);

% run simulation
for i = 2:N
    % advance timestep
    state.setTime(t(i));

    % TODO: calculate phi (u(t))
    % based on adaptive/robust control policy
    

    % set joint angle according to above policy
    model.getJointSet().get(1).getCoordinate().setValue(state,u(i));

    % let muscle come to equilibrium
    model.equilibrateMuscles(state);
    
    x(i) = model.getMuscles().get(0).getFiberLength(state);
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