%% Double Pendulum Simulator
% This script simulates the chaotic motion of a double pendulum system
% using numerical integration of the equations of motion.
%
% The double pendulum consists of two point masses connected by massless
% rigid rods. The system exhibits chaotic behavior for certain initial
% conditions, demonstrating sensitive dependence on initial conditions.
%
% Author: Soumik Saha
% Institution: Bangladesh University of Engineering and Technology (BUET)
% Course: Numerical Methods / Dynamics
% Date: March 2023

clc;
clear all;
close all;

%% System Parameters

% Physical properties
m1 = 2;      % Mass of first pendulum (kg)
m2 = 1;      % Mass of second pendulum (kg)
l1 = 1;      % Length of first rod (m)
l2 = 2;      % Length of second rod (m)
g = 9.8;     % Gravitational acceleration (m/s^2)

% Pack parameters for ODE solver
system_params = [m1; m2; l1; l2; g];

%% Initial Conditions

% Angular positions (radians)
theta1_0 = 2.2;     % Initial angle of first pendulum
theta2_0 = 1.6;     % Initial angle of second pendulum

% Angular velocities (rad/s)
omega1_0 = 0;       % Initial angular velocity of first pendulum
omega2_0 = 0;       % Initial angular velocity of second pendulum

% Pack initial conditions
initial_state = [theta1_0; theta2_0; omega1_0; omega2_0];

%% Simulation Parameters

% Time span for simulation
t_start = 0;
t_end = 100;
dt = 0.03;          % Time step (s)
timespan = t_start:dt:t_end;

%% Solve Equations of Motion

% Use ODE45 (Runge-Kutta 4th/5th order) to solve the system
fprintf('Starting simulation...\n');
[t, results] = ode45(@(t, theta) odefunction(t, theta, initial_state, system_params), ...
                     timespan, initial_state);
fprintf('Simulation complete!\n');

%% Extract Results

% Angular positions
theta1 = results(:, 1);     % Angle of first pendulum
theta2 = results(:, 2);     % Angle of second pendulum

% Angular velocities
omega1 = results(:, 3);     % Angular velocity of first pendulum
omega2 = results(:, 4);     % Angular velocity of second pendulum

%% Convert to Cartesian Coordinates

% Position of first mass
x1 = l1 * sin(theta1);
y1 = -l1 * cos(theta1);

% Position of second mass (relative to origin)
x2 = l1 * sin(theta1) + l2 * sin(theta2);
y2 = -l1 * cos(theta1) - l2 * cos(theta2);

%% Animate the Motion

fprintf('Animating double pendulum motion...\n');
figure('Position', [100, 100, 800, 600]);

for i = 1:length(t)
    clf;
    
    % Fixed point (origin)
    x0 = 0;
    y0 = 0;
    
    % Plot horizontal reference line
    plot([-1 1], [0 0], 'color', 'k', 'LineWidth', 1);
    hold on;
    
    % Plot first rod
    plot([x0 x1(i)], [y0 y1(i)], 'color', 'k', 'LineWidth', 2);
    
    % Plot second rod
    plot([x1(i) x2(i)], [y1(i) y2(i)], 'color', 'k', 'LineWidth', 2);
    
    % Plot masses
    plot(x1(i), y1(i), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    plot(x2(i), y2(i), 'ro', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    
    % Plot trajectory traces
    plot(x1(1:i), y1(1:i), 'LineWidth', 1, 'Color', [0.8 0.2 0.2], 'LineStyle', '--');
    plot(x2(1:i), y2(1:i), 'LineWidth', 1, 'Color', [0 0.447 0.741], 'LineStyle', '--');
    
    % Set axis properties
    axis equal;
    axis([-3 3 -3 3]);
    grid on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(sprintf('Double Pendulum Motion (t = %.2f s)', t(i)));
    legend('Reference', 'Rod 1', 'Rod 2', 'Mass 1', 'Mass 2', 'Trace 1', 'Trace 2', ...
           'Location', 'northeast');
    
    pause(0.01);
end

fprintf('Animation complete!\n');

%% Calculate Energy

fprintf('Calculating system energy...\n');

% Potential Energy
PE1 = m1 * g * y1;                  % PE of first mass
PE2 = m2 * g * y2;                  % PE of second mass

% Kinetic Energy
KE1 = 0.5 * m1 * l1^2 * omega1.^2;  % KE of first mass
KE2 = 0.5 * m2 * ((l1^2 * omega1.^2) + (l2^2 * omega2.^2) + ...
                  (2 * l1 * l2 * omega1 .* omega2 .* cos(theta1 - theta2)));

% Total Energy
E1 = PE1 + KE1;                     % Total energy of first mass
E2 = PE2 + KE2;                     % Total energy of second mass
E_total = E1 + E2;                  % Total system energy

%% Animate Energy Plot

fprintf('Animating energy evolution...\n');
figure('Position', [100, 100, 1000, 600]);

for i = 1:5:length(t)
    clf;
    
    % Plot current energy points
    plot(t(i), E1(i), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    hold on;
    plot(t(i), E2(i), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot(t(i), E_total(i), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    
    % Plot energy history
    plot(t(1:i), E1(1:i), 'LineWidth', 2, 'Color', 'r', 'LineStyle', '-');
    plot(t(1:i), E2(1:i), 'LineWidth', 2, 'Color', 'g', 'LineStyle', '-');
    plot(t(1:i), E_total(1:i), 'LineWidth', 2, 'Color', 'b', 'LineStyle', '-');
    
    % Set axis properties
    axis([0 100 -100 100]);
    grid on;
    xlabel('Time (s)');
    ylabel('Energy (J)');
    title('Energy Evolution of Double Pendulum System');
    legend('Mass 1 Energy', 'Mass 2 Energy', 'Total Energy', ...
           'E1(t)', 'E2(t)', 'E_{total}(t)', 'Location', 'best');
    
    pause(0.0001);
end

fprintf('Energy animation complete!\n');

%% Generate Summary Plots

figure('Position', [100, 100, 1200, 800]);

% Angular positions
subplot(2, 2, 1);
plot(t, theta1, 'r-', 'LineWidth', 1.5);
hold on;
plot(t, theta2, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Angular Positions');
legend('\theta_1', '\theta_2');
grid on;

% Angular velocities
subplot(2, 2, 2);
plot(t, omega1, 'r-', 'LineWidth', 1.5);
hold on;
plot(t, omega2, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocities');
legend('\omega_1', '\omega_2');
grid on;

% Phase space (Poincaré section)
subplot(2, 2, 3);
plot(theta1, omega1, 'r.', 'MarkerSize', 2);
hold on;
plot(theta2, omega2, 'b.', 'MarkerSize', 2);
xlabel('Angle (rad)');
ylabel('Angular Velocity (rad/s)');
title('Phase Space');
legend('Mass 1', 'Mass 2');
grid on;

% Energy conservation
subplot(2, 2, 4);
plot(t, E_total, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Total Energy (J)');
title('Total System Energy');
grid on;

fprintf('\n=== Simulation Summary ===\n');
fprintf('Mass 1: %.2f kg, Length 1: %.2f m\n', m1, l1);
fprintf('Mass 2: %.2f kg, Length 2: %.2f m\n', m2, l2);
fprintf('Initial angles: θ1 = %.2f rad, θ2 = %.2f rad\n', theta1_0, theta2_0);
fprintf('Simulation time: %.2f s\n', t_end);
fprintf('Energy conservation error: %.4f J\n', max(abs(E_total - E_total(1))));
fprintf('========================\n');
