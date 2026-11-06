%% build_peltier_simulink.m
% Builds two Simulink models for the Peltier surface-temperature plant:
%   1) peltier_open_loop    -> response BEFORE the controller
%   2) peltier_closed_loop  -> response AFTER adding the PID controller
% Run this script, then press Run in each model.

clc; clear; close all;

%% Plant parameters (illustrative - replace with measured values)
K_surface   = 15;     % degC per unit normalised PWM
tau_surface = 180;    % s
Kp = 0.2;  Ki = 0.002;  Kd = 0;   % starting PID gains - tune these

%% ---------- 1) OPEN-LOOP MODEL ----------
mdl = 'peltier_open_loop';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Step',            [mdl '/PWM Command'], ...
    'Time','0','Before','0','After','0.5');            % 50 % duty step
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Peltier Surface G(s)'], ...
    'Numerator','[-K_surface]','Denominator','[tau_surface 1]');
add_block('simulink/Sinks/Scope',             [mdl '/Surface Temp Change']);

add_line(mdl,'PWM Command/1','Peltier Surface G(s)/1');
add_line(mdl,'Peltier Surface G(s)/1','Surface Temp Change/1');
set_param(mdl,'StopTime','1000');

try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
save_system(mdl);

%% ---------- 2) CLOSED-LOOP MODEL ----------
mdl = 'peltier_closed_loop';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);

% Setpoint = desired temperature CHANGE (cool by 10 degC)
add_block('simulink/Sources/Step', [mdl '/Setpoint'], ...
    'Time','0','Before','0','After','-10');

% Cooling is reverse-acting (more PWM -> lower temp), so error = y - r
add_block('simulink/Math Operations/Sum', [mdl '/Error'], ...
    'Inputs','-+');

add_block('simulink/Continuous/PID Controller', [mdl '/PID'], ...
    'P','Kp','I','Ki','D','Kd', ...
    'LimitOutput','on','UpperSaturationLimit','1', ...
    'LowerSaturationLimit','0','AntiWindupMode','clamping');

add_block('simulink/Continuous/Transfer Fcn', [mdl '/Peltier Surface G(s)'], ...
    'Numerator','[-K_surface]','Denominator','[tau_surface 1]');

% Disturbance (e.g. heat load) added at the plant output
add_block('simulink/Sources/Step', [mdl '/Disturbance'], ...
    'Time','600','Before','0','After','2');
add_block('simulink/Math Operations/Sum', [mdl '/Dist Sum'], ...
    'Inputs','++');

add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'NumInputPorts','2');

add_line(mdl,'Setpoint/1','Error/1');
add_line(mdl,'Error/1','PID/1');
add_line(mdl,'PID/1','Peltier Surface G(s)/1');
add_line(mdl,'Peltier Surface G(s)/1','Dist Sum/1');
add_line(mdl,'Disturbance/1','Dist Sum/2');
add_line(mdl,'Dist Sum/1','Error/2');      % feedback path (measured temp)
add_line(mdl,'Dist Sum/1','Scope/1');
add_line(mdl,'PID/1','Scope/2');           % control effort (PWM)
set_param(mdl,'StopTime','1500');

try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
save_system(mdl);

disp('Models created: peltier_open_loop, peltier_closed_loop');
