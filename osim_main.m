% simulation code for me533 final project/term paper
% simulate joint dynamics model (2nd order mass/spring/damper)
% using OpenSim arm26 model

clc;close all;clear variables;

% import opensim packages
import org.opensim.modeling.*;

% initialize model
model = Model('arm26.osim');

% add actuator for external torque
actuator = CoordinateActuator();
actuator.setName('elbowActuator');
actuator.setCoordinate(model.getCoordinateSet().get('r_elbow_flex'));
actuator.setOptimalForce(1);
model.addForce(actuator);

state = model.initSystem();
state.setTime(0);


% disable gravity
model.setGravity(Vec3(0,0,0));

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

% get initial state x0
x(1) = model.getJointSet().get(1).getCoordinate().getValue(state);
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

% unlock elbow joint
model.getJointSet.get(1).getCoordinate().setLocked(state, false);

% run simulation
for i = 2:N
    % advance timestep
    state.setTime(t(i));

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

    % set external force according to control law above
    model.updActuators().get('elbowActuator').addInControls(Vector(1,u(i)), model.updControls(state));

    % perform forward simulation and let muscles come to equilibrium
    model.realizeDynamics(state);
    model.realizeVelocity(state);
    % model.equilibrateMuscles(state);

    % compute x and x_dot
    x(i) = model.getJointSet().get(1).getCoordinate().getValue(state);
    x_dot(i) = model.getJointSet().get(1).getCoordinate().getSpeedValue(state);

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