%% Runpos_control.m
% This script runs a simulation of a motor voltage and wheel speed and pos
% and plots the results.
% 
% Required file: position_control.slx
% 
% File for simulink block diagram with proportional control
%
%load('controlData.mat')
%% Define Motor Parameters
K=1.75; % DC gain [rad/Vs]
sigma=17.5; % time constant reciprocal [1/s]
omega_d=3; % desired velocity [rad/s]
Kp=2; % Proportional gain
%% Run a Simulation
open_system('motor_control');
in = Simulink.SimulationInput('motor_control');
% declare variables to pass between matlab and simulink
in = in.setVariable('K', K, 'Workspace', 'motor_control');
in = in.setVariable('sigma', sigma, 'Workspace', 'motor_control');
in = in.setVariable('omega_d', omega_d, 'Workspace', 'motor_control');
in = in.setVariable('Kp', Kp, 'Workspace', 'motor_control');
out = sim(in);
%% Plot of the Results
% Plot of voltage vs time with simulated data vs experimental data
% Interpretation: The voltage vs time graph shows a spike at 1 second
% as the difference calculation has the largest differences and thus
% produces a large spike. The same trend continues during the 3 second
% trial as the motor voltage oscillates. Both the experimental and
% simulated data settle around 2.5V.
%
% Plot of velocity vs time with simulated data vs experimental data
% Interpretation: The velocity vs time graph varies for the simulated vs
% experimental data. For the simulated data, the velocity grows smoothly
% with no overshoot and settles at 4 rad/s when the desired speed is set
% to 5 rad/s. For the experimental data, the graph spikes at 1s similar to
% the behavior seen in the voltage vs time graph, overshoots, oscillates
% slightly then settles around 4.8 rad/s.

% Voltage vs time plot
figure
subplot(2,1,1)
plot(out.Voltage,'--','linewidth',2)
hold on
plot(data2(:,1),data2(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
% Velocity vs time plot
subplot(2,1,2)
plot(out.Velocity,'linewidth',2)
hold on
plot(data2(:,1), data2(:,3),'--','linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')