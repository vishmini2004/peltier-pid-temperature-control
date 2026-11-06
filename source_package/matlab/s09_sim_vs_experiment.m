%% S09_SIM_VS_EXPERIMENT  Re-simulate a logged closed-loop run and compare
% Report section 12. The logged setpoint and heater (load) signals are replayed
% into the model with the identified parameters and the SAME gains as the firmware,
% so any difference is model error, not a different test.
%
% Inputs
%   ../data/closedloop_run.csv      firmware log: t_s,T_surf,T_hot,T_ref,duty,load,mode
%   ../data/identified_params.mat   from s08 (falls back to peltier_params if missing)
%   d_heater                        heater load in duty units = open-loop rise with the
%                                   heater on / K  (measure it, see report section 10)
clear; clc; close all;
here = fileparts(mfilename('fullpath'));

% ---- gains: MUST equal the values running in the firmware ------------------
P = peltier_params();
S = pid_designs(P);
g = S(strcmp({S.name}, 'PID_recommended'));
d_heater = 0.2;                  % PROVISIONAL - replace with the measured heater effect

file = fullfile(here, '..', 'data', 'closedloop_run.csv');
synthetic = ~isfile(file);
if synthetic
    warning('No measured closed-loop log found - using SYNTHETIC data to exercise the script only.');
    make_synthetic_data();
    file = fullfile(here, '..', 'data', 'SYNTHETIC_closedloop_run.csv');
end
idf = fullfile(here, '..', 'data', 'identified_params.mat');
if isfile(idf)
    I = load(idf); P.K = I.K; P.tau = I.tau; P.L = I.L;
    fprintf('Using identified parameters: K = %.2f, tau = %.0f s, L = %.1f s%s\n', P.K, P.tau, P.L, ...
            repmat(' (SYNTHETIC)', 1, I.synthetic));
else
    fprintf('identified_params.mat not found - using PROVISIONAL parameters.\n');
end

D = readtable(file);
t = D.t_s - D.t_s(1);
P.Tamb  = D.T_surf(1);           % run must start from ambient with the Peltier off
P.t_end = t(end);
ref  = @(q) interp1(t, D.T_ref, q, 'previous', 'extrap');
dist = @(q) d_heater*interp1(t, double(D.load), q, 'previous', 'extrap');
o = simulate_loop(P, g, struct('ref', ref, 'dist', dist));

Tsim = interp1(o.t, o.Tm, t);    % simulated MEASURED temperature on the log's time grid
usim = interp1(o.t, o.u,  t);
err  = D.T_surf - Tsim;
fprintf('\nRMSE(T) = %.3f degC, max |error| = %.3f degC, RMSE(duty) = %.3f\n', ...
        sqrt(mean(err.^2, 'omitnan')), max(abs(err)), sqrt(mean((D.duty - usim).^2, 'omitnan')));

% ---- metrics for every setpoint change in the log, side by side --------------
kc = [1; find(abs(diff(D.T_ref)) > 0.01) + 1; numel(t) + 1];
fprintf('\n%-14s | %-26s | %-26s\n', 'Step', 'Experiment tr / os% / ts', 'Simulation tr / os% / ts');
for k = 1:numel(kc) - 1
    a = kc(k); b = kc(k+1) - 1;
    r = D.T_ref(a); T0 = D.T_surf(max(a-1, 1)); if k == 1, T0 = P.Tamb; end
    if abs(r - T0) < 1, continue; end
    me = step_metrics(t, D.T_surf, r, T0, t(a), t(b));
    ms = step_metrics(t, Tsim,     r, T0, t(a), t(b));
    fprintf('%5.1f -> %-5.1f | %6.0f / %5.2f / %6.0f     | %6.0f / %5.2f / %6.0f\n', T0, r, ...
            me.tr, me.os, me.ts, ms.tr, ms.os, ms.ts);
end

fig = figure('Name', 'Sim vs experiment', 'Color', 'w', 'Position', [100 100 900 750]);
subplot(3,1,1); hold on; grid on;
plot(t, D.T_ref, 'k--', 'DisplayName', 'Setpoint');
plot(t, D.T_surf, 'LineWidth', 1.2, 'DisplayName', 'Experiment');
plot(t, Tsim, 'LineWidth', 1.2, 'DisplayName', 'Simulation');
ylabel('T (\circC)'); legend;
title(ifelse(synthetic, 'SYNTHETIC data - pipeline test only', 'Simulation vs experiment'));
subplot(3,1,2); hold on; grid on;
plot(t, D.duty, 'DisplayName', 'Experiment'); plot(t, usim, 'DisplayName', 'Simulation');
ylabel('Duty'); legend;
subplot(3,1,3); grid on; plot(t, err); ylabel('T_{exp} - T_{sim} (\circC)'); xlabel('Time (s)');
save_fig(fig, 's09_sim_vs_experiment');

function out = ifelse(c, a, b)
if c, out = a; else, out = b; end
end
