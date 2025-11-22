function [derivatives] = odefunction(t, state, initial_conditions, system_params)
%ODEFUNCTION Equations of motion for double pendulum system
%
% This function computes the derivatives (angular velocities and
% accelerations) for a double pendulum system using the Lagrangian
% formulation of classical mechanics.
%
% Inputs:
%   t                  - Current time (unused, but required by ODE solver)
%   state              - Current state vector [theta1; theta2; omega1; omega2]
%   initial_conditions - Initial state (unused in current implementation)
%   system_params      - System parameters [m1; m2; l1; l2; g]
%
% Outputs:
%   derivatives - Time derivatives [dtheta1/dt; dtheta2/dt; domega1/dt; domega2/dt]
%
% The equations are derived from the Lagrangian:
%   L = T - V
% where T is kinetic energy and V is potential energy.
%
% Author: Soumik Saha
% Date: March 2023

%% Extract System Parameters

m1 = system_params(1);  % Mass of first pendulum (kg)
m2 = system_params(2);  % Mass of second pendulum (kg)
l1 = system_params(3);  % Length of first rod (m)
l2 = system_params(4);  % Length of second rod (m)
g  = system_params(5);  % Gravitational acceleration (m/s^2)

%% Extract Current State

theta1 = state(1);      % Angle of first pendulum (rad)
theta2 = state(2);      % Angle of second pendulum (rad)
omega1 = state(3);      % Angular velocity of first pendulum (rad/s)
omega2 = state(4);      % Angular velocity of second pendulum (rad/s)

%% Calculate Angular Accelerations

% These equations come from solving the Euler-Lagrange equations
% for a double pendulum system. They describe the angular accelerations
% as functions of the current angles and angular velocities.

% Difference in angles
delta_theta = theta1 - theta2;

% Denominators for the angular accelerations
denom1 = (m1 + m2) * l1 - m2 * l1 * (cos(delta_theta))^2;
denom2 = l2 - (m2 * l2 / (m1 + m2)) * (cos(delta_theta))^2;

% Angular acceleration of first pendulum
alpha1 = (1 / denom1) * ...
         (-m2 * l1 * omega1^2 * sin(delta_theta) * cos(delta_theta) + ...
          m2 * g * sin(theta2) * cos(delta_theta) - ...
          m2 * l2 * omega2^2 * sin(delta_theta) - ...
          (m1 + m2) * g * sin(theta1));

% Angular acceleration of second pendulum
alpha2 = (1 / denom2) * ...
         (l1 * omega1^2 * sin(delta_theta) - ...
          g * sin(theta2) + ...
          (m2 * l2 / (m1 + m2)) * omega2^2 * sin(delta_theta) * cos(delta_theta) + ...
          g * sin(theta1) * cos(delta_theta));

%% Assemble Derivatives Vector

% State vector derivatives:
% d/dt [theta1]   [omega1]
%      [theta2] = [omega2]
%      [omega1]   [alpha1]
%      [omega2]   [alpha2]

derivatives = zeros(4, 1);
derivatives(1) = omega1;    % dtheta1/dt = omega1
derivatives(2) = omega2;    % dtheta2/dt = omega2
derivatives(3) = alpha1;    % domega1/dt = alpha1
derivatives(4) = alpha2;    % domega2/dt = alpha2

end
