%% run_peltier_simulink.m
% Builds, runs and plots the Simulink models of the Peltier surface:
%   peltier_open_loop    - Step -> Transfer Fcn -> Scope
%   peltier_closed_loop  - Step -> Sum -> PID -> Transfer Fcn -> Sum -> Scope
% Run the whole script (F5). Needs MATLAB + Simulink (Control System Toolbox
% is only needed for tf/step in section 1 and for the overlay check).

clc; clear; close all;

%% 1. Parameters (replace K and tau with your measured values)
K     = 15;        % degC per unit duty
tau   = 180;       % s
Tamb  = 25;        % degC (initial surface temperature)
Kp    = 0.3;  Ki = 0.004;  Kd = 5;   % PID gains (replace with your tuned gains)
Tset  = 15;        % target surface temperature (degC)
dist  = 3;         % disturbance size (degC)
tdist = 1200;      % disturbance time (s)

% Transfer function and state-space model in the workspace
s = tf('s');
G = -K/(tau*s + 1)
[A,B,C,D] = ssdata(ss(G))

%% 2. OPEN-LOOP MODEL  (before the controller)
mdl = 'peltier_open_loop';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Step',[mdl '/PWM_Step'], ...
    'Time','0','Before','0','After','0.5', ...
    'Position',[40 60 90 100]);
add_block('simulink/Continuous/Transfer Fcn',[mdl '/Peltier_TF'], ...
    'Numerator',mat2str(-K),'Denominator',mat2str([tau 1]), ...
    'Position',[160 55 300 105]);
add_block('simulink/Sinks/Scope',[mdl '/Scope'], ...
    'Position',[380 55 420 105]);
add_block('simulink/Sinks/To Workspace',[mdl '/ToWS'], ...
    'VariableName','dT_open','SaveFormat','Timeseries', ...
    'Position',[380 140 440 170]);

add_line(mdl,'PWM_Step/1','Peltier_TF/1');
add_line(mdl,'Peltier_TF/1','Scope/1');
add_line(mdl,'Peltier_TF/1','ToWS/1');
set_param(mdl,'StopTime','1000');
save_system(mdl);

outOpen = sim(mdl,'ReturnWorkspaceOutputs','on');
dT = outOpen.get('dT_open');

%% 3. CLOSED-LOOP MODEL  (after the controller)
mdl = 'peltier_closed_loop';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

% Work with absolute temperature: plant output = Tamb + dT + disturbance
add_block('simulink/Sources/Step',[mdl '/Setpoint'], ...
    'Time','0','Before',num2str(Tamb),'After',num2str(Tset), ...
    'Position',[30 60 80 100]);
add_block('simulink/Math Operations/Sum',[mdl '/Error'], ...
    'IconShape','round','Inputs','-+', ...
    'Position',[130 70 150 90]);               % e = y - r  (reverse-acting)
add_block('simulink/Continuous/PID Controller',[mdl '/PID'], ...
    'P',num2str(Kp),'I',num2str(Ki),'D',num2str(Kd),'N','0.1', ...
    'LimitOutput','on','UpperSaturationLimit','1', ...
    'LowerSaturationLimit','0','AntiWindupMode','clamping', ...
    'Position',[200 55 290 105]);
add_block('simulink/Continuous/Transfer Fcn',[mdl '/Peltier_TF'], ...
    'Numerator',mat2str(-K),'Denominator',mat2str([tau 1]), ...
    'Position',[350 55 470 105]);
add_block('simulink/Sources/Constant',[mdl '/Ambient'], ...
    'Value',num2str(Tamb),'Position',[350 130 400 160]);
add_block('simulink/Sources/Step',[mdl '/Disturbance'], ...
    'Time',num2str(tdist),'Before','0','After',num2str(dist), ...
    'Position',[350 190 400 220]);
add_block('simulink/Math Operations/Sum',[mdl '/PlantSum'], ...
    'IconShape','round','Inputs','+++', ...
    'Position',[530 65 560 95]);
add_block('simulink/Sinks/Scope',[mdl '/Scope'], ...
    'NumInputPorts','2','Position',[640 55 680 115]);
add_block('simulink/Sinks/To Workspace',[mdl '/ToWS_T'], ...
    'VariableName','T_closed','SaveFormat','Timeseries', ...
    'Position',[640 150 700 180]);
add_block('simulink/Sinks/To Workspace',[mdl '/ToWS_u'], ...
    'VariableName','u_closed','SaveFormat','Timeseries', ...
    'Position',[640 200 700 230]);

add_line(mdl,'Setpoint/1','Error/1');
add_line(mdl,'Error/1','PID/1');
add_line(mdl,'PID/1','Peltier_TF/1');
add_line(mdl,'Peltier_TF/1','PlantSum/1');
add_line(mdl,'Ambient/1','PlantSum/2');
add_line(mdl,'Disturbance/1','PlantSum/3');
add_line(mdl,'PlantSum/1','Error/2');         % feedback: measured temperature
add_line(mdl,'PlantSum/1','Scope/1');
add_line(mdl,'PID/1','Scope/2');              % control duty
add_line(mdl,'PlantSum/1','ToWS_T/1');
add_line(mdl,'PID/1','ToWS_u/1');
set_param(mdl,'StopTime','2400');
save_system(mdl);

outClosed = sim(mdl,'ReturnWorkspaceOutputs','on');
Tc = outClosed.get('T_closed');
uc = outClosed.get('u_closed');

%% 4. Plots for the report
t = (0:1:1000)';
figure('Name','Open loop');
plot(dT.Time, dT.Data, 'LineWidth',1.5); hold on;
plot(t, -K*0.5*(1-exp(-t/tau)), 'r--');          % analytical check
grid on; xlabel('Time (s)'); ylabel('\DeltaT_{surface} (°C)');
legend('Simulink','Analytical'); title('Open-loop response, duty step = 0.5');

figure('Name','Closed loop');
subplot(2,1,1);
plot(Tc.Time, Tc.Data, 'LineWidth',1.5); hold on;
yline(Tset,'k--'); xline(tdist,'m:'); grid on;
ylabel('Surface temperature (°C)');
title(sprintf('PID closed loop, setpoint = %g °C', Tset));
subplot(2,1,2);
plot(uc.Time, uc.Data, 'LineWidth',1.5); grid on;
ylabel('Duty (0-1)'); xlabel('Time (s)');

disp('Done. Models: peltier_open_loop and peltier_closed_loop.');
