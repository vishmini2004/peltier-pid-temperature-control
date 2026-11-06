function build_simulink_model(design_name)
%BUILD_SIMULINK_MODEL  Create peltier_pid_model.slx programmatically, run it and plot.
%   build_simulink_model                       % recommended PID
%   build_simulink_model('PI_SIMC')            % any name from pid_designs
%
%   Corrected architecture (see report Appendix B/C):
%     Ref --(-)--+                                   Tamb --+
%                Err -> PID[0,1] -> DistSum(+ u, - d_u) -> Plant -K/(tau s+1) -> TempSum(+ +)
%     T_meas --(+)+                                                                 |
%        ^                                                                     T_surface
%        +-- Quantizer <- ZOH(Ts) <- Transport Delay (L) <---------------------------+
%
%   Error convention: e = T_meas - T_ref (reverse acting), so the PID gains are
%   positive even though the plant gain is negative. Clamping anti-windup, output
%   limited to [0, 1].
%
%   Written against the documented Simulink block parameters (R2021a or later).
%   It was NOT run by the author of this package - if a parameter name is
%   rejected by your release, set it by hand in the block dialog (App. C lists them).

if nargin < 1, design_name = 'PID_recommended'; end
P = peltier_params();
S = pid_designs(P);
g = S(strcmp({S.name}, design_name));
assignin('base', 'P', P);
assignin('base', 'g', g);

mdl = 'peltier_pid_model';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% ---------------- blocks --------------------------------------------------------
add_block('simulink/Sources/Step', [mdl '/Ref'], 'Time', '0', 'Before', 'P.Tamb', ...
          'After', 'P.T_ref', 'Position', [40 60 70 90]);
add_block('simulink/Math Operations/Sum', [mdl '/Err'], 'Inputs', '+-', ...
          'IconShape', 'round', 'Position', [140 95 160 115]);
add_block('simulink/Discrete/Discrete PID Controller', [mdl '/PID'], ...
          'Controller', 'PID', 'Form', 'Parallel', ...
          'P', 'g.Kp', 'I', 'g.Ki', 'D', 'g.Kd', 'N', '1/g.Tf', ...
          'SampleTime', 'P.Ts', 'IntegratorMethod', 'Backward Euler', ...
          'FilterMethod', 'Backward Euler', ...
          'LimitOutput', 'on', 'UpperSaturationLimit', 'P.umax', 'LowerSaturationLimit', 'P.umin', ...
          'AntiWindupMode', 'clamping', 'Position', [200 85 270 125]);
add_block('simulink/Sources/Step', [mdl '/Disturb'], 'Time', 'P.t_load', 'Before', '0', ...
          'After', 'P.d_load', 'Position', [240 170 270 200]);
add_block('simulink/Math Operations/Sum', [mdl '/DistSum'], 'Inputs', '+-', ...
          'IconShape', 'round', 'Position', [320 95 340 115]);
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant'], 'Numerator', '[-P.K]', ...
          'Denominator', '[P.tau 1]', 'Position', [380 85 460 125]);
add_block('simulink/Sources/Constant', [mdl '/Tamb'], 'Value', 'P.Tamb', 'Position', [420 20 460 40]);
add_block('simulink/Math Operations/Sum', [mdl '/TempSum'], 'Inputs', '++', ...
          'IconShape', 'round', 'Position', [500 95 520 115]);
add_block('simulink/Continuous/Transport Delay', [mdl '/Delay'], 'DelayTime', 'P.L', ...
          'InitialOutput', 'P.Tamb', 'Position', [520 230 570 260], 'Orientation', 'left');
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Sample'], 'SampleTime', 'P.Ts', ...
          'Position', [400 230 430 260], 'Orientation', 'left');
add_block('simulink/Discontinuities/Quantizer', [mdl '/Quantizer'], 'QuantizationInterval', 'P.q', ...
          'Position', [280 230 310 260], 'Orientation', 'left');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], 'Inputs', '3', 'Position', [640 40 645 120]);
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'NumInputPorts', '2', 'Position', [700 60 730 110]);
logs = {'T_surface', 'TempSum'; 'T_meas', 'Quantizer'; 'duty', 'PID'; 'T_ref', 'Ref'};
for i = 1:size(logs,1)
    add_block('simulink/Sinks/To Workspace', [mdl '/log_' logs{i,1}], 'VariableName', logs{i,1}, ...
              'SaveFormat', 'Timeseries', 'Position', [700 140+45*i 760 160+45*i]);
end

% ---------------- wiring --------------------------------------------------------
L = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
L('Quantizer/1', 'Err/1');      % + T_meas
L('Ref/1',       'Err/2');      % - T_ref     -> e = T_meas - T_ref
L('Err/1',       'PID/1');
L('PID/1',       'DistSum/1');  % + u
L('Disturb/1',   'DistSum/2');  % - d_u (heat load cancels part of the pumping)
L('DistSum/1',   'Plant/1');
L('Tamb/1',      'TempSum/1');
L('Plant/1',     'TempSum/2');  % T_surface = T_amb + Delta T (Delta T < 0 when cooling)
L('TempSum/1',   'Delay/1');
L('Delay/1',     'Sample/1');
L('Sample/1',    'Quantizer/1');
L('TempSum/1',   'Mux/1');
L('Quantizer/1', 'Mux/2');
L('Ref/1',       'Mux/3');
L('Mux/1',       'Scope/1');
L('PID/1',       'Scope/2');
for i = 1:size(logs,1), L([logs{i,2} '/1'], ['log_' logs{i,1} '/1']); end

% ---------------- solver ----------------------------------------------------------
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', num2str(P.dt), ...
          'StopTime', num2str(P.t_end));
here = fileparts(mfilename('fullpath'));
save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Saved %s.slx (design %s)\n', mdl, design_name);

% ---------------- run and compare with the MATLAB simulation ---------------------
out = sim(mdl);
Ts_ = out.T_surface; u_ = out.duty;
o = simulate_loop(P, g);
fig = figure('Name', 'Simulink vs script', 'Color', 'w');
subplot(2,1,1); hold on; grid on;
plot(Ts_.Time, squeeze(Ts_.Data), 'LineWidth', 1.4, 'DisplayName', 'Simulink');
plot(o.t, o.T, '--', 'LineWidth', 1.2, 'DisplayName', 'simulate\_loop.m');
yline(P.T_ref, 'k:'); ylabel('T_{surface} (\circC)'); legend;
title(['Simulink model vs MATLAB script, ' strrep(design_name, '_', ' ')]);
subplot(2,1,2); hold on; grid on;
plot(u_.Time, squeeze(u_.Data)); plot(o.t, o.u, '--'); ylabel('Duty'); xlabel('Time (s)');
save_fig(fig, 'simulink_vs_script');
end
