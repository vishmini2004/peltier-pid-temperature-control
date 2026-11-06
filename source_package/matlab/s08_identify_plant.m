%% S08_IDENTIFY_PLANT  Identify K, tau and L from an open-loop duty step
% This is how the PROVISIONAL values in peltier_params.m become MEASURED values.
%
% Experiment (firmware command sequence, see report section 10):
%   MODE OPEN ; DUTY 0      -> wait until T_surf changes < 0.1 degC in 2 min
%   DUTY 0.6                -> log for at least 5*tau (about 15-20 min)
%   (repeat with DUTY 0.3 and DUTY 0.9 to check linearity: K should agree within ~10 %)
% Expected CSV (written by log_serial.m): t_s,T_surf,T_hot,T_ref,duty,load,mode
%
% Output: ../data/identified_params.mat (K, tau, L, Tamb) used by s09.
clear; clc; close all;
here = fileparts(mfilename('fullpath'));
file = fullfile(here, '..', 'data', 'openloop_step.csv');
synthetic = ~isfile(file);
if synthetic
    warning(['No measured data at %s.\nUsing SYNTHETIC data only to exercise the script - ' ...
             'do NOT report these numbers.'], file);
    make_synthetic_data();
    file = fullfile(here, '..', 'data', 'SYNTHETIC_openloop_step.csv');
end
D = readtable(file);
t = D.t_s - D.t_s(1); T = D.T_surf; u = D.duty;

% ---- locate the step and the steady states ----------------------------------
k0 = find(abs(diff(u)) > 0.05, 1) + 1;                  % first sample after the step
if isempty(k0), error('No duty step found in %s', file); end
t0 = t(k0); du = u(k0) - u(k0-1);
T0   = mean(T(max(1, k0-30):k0-1));                     % last 30 s before the step
tend = t(end);
Tinf = mean(T(t >= tend - 0.1*(tend - t0)));            % last 10 % of the record
K0 = (T0 - Tinf)/du;                                    % cooling gain, degC per unit duty

% ---- two-point method (Smith): 28.3 % and 63.2 % of the change --------------
tt = t(k0:end) - t0;
y  = (T0 - T(k0:end))/(T0 - Tinf);
t28 = tt(find(y >= 0.283, 1)); t63 = tt(find(y >= 0.632, 1));
tau0 = 1.5*(t63 - t28);
L0   = max(0, t63 - tau0);

% ---- least-squares refinement of the FOPDT step response --------------------
model = @(p, s) (s > p(3)).*p(1)*du.*(1 - exp(-max(s - p(3), 0)/p(2)));
cost  = @(p) sum((T0 - T(k0:end) - model(p, tt)).^2);
p = fminsearch(cost, [K0 tau0 L0], optimset('Display', 'off', 'MaxFunEvals', 5000));
Kid = p(1); tauid = p(2); Lid = max(0, p(3));
res = (T0 - T(k0:end)) - model(p, tt);
fit = 100*(1 - norm(res)/norm((T0 - T(k0:end)) - mean(T0 - T(k0:end))));   % NRMSE fit %

fprintf('Data file          : %s %s\n', file, ternary(synthetic, '(SYNTHETIC)', '(measured)'));
fprintf('Duty step          : %.2f at t = %.0f s\n', du, t0);
fprintf('T before / final   : %.2f / %.2f degC\n', T0, Tinf);
fprintf('Two-point estimate : K = %.2f, tau = %.0f s, L = %.1f s\n', K0, tau0, L0);
fprintf('Least squares      : K = %.2f, tau = %.0f s, L = %.1f s  (fit %.1f %%)\n', Kid, tauid, Lid, fit);
if ismember('T_hot', D.Properties.VariableNames)
    fprintf('Hot-side rise      : %.1f degC (if > ~5 degC the heat sink adds a slow second mode)\n', ...
            max(D.T_hot) - D.T_hot(1));
end

fig = figure('Name', 'Identification', 'Color', 'w', 'Position', [100 100 900 600]);
subplot(2,1,1); hold on; grid on;
plot(t, T, '.', 'MarkerSize', 4, 'DisplayName', 'Measured');
plot(t0 + tt, T0 - model(p, tt), 'LineWidth', 1.6, 'DisplayName', ...
     sprintf('FOPDT fit: K=%.1f, \\tau=%.0f s, L=%.1f s', Kid, tauid, Lid));
ylabel('T_{surface} (\circC)'); legend; title(ternary(synthetic, 'SYNTHETIC data - pipeline test only', ...
     'Open-loop step identification'));
subplot(2,1,2); grid on;
plot(t0 + tt, res); ylabel('Residual (\circC)'); xlabel('Time (s)');
title('Residual: a slow trend here means the first-order model misses heat-sink dynamics');
save_fig(fig, 's08_identification');

ident = struct('K', Kid, 'tau', tauid, 'L', Lid, 'Tamb', T0, 'synthetic', synthetic, 'source', file);
save(fullfile(here, '..', 'data', 'identified_params.mat'), '-struct', 'ident');
fprintf('\nSaved ../data/identified_params.mat. Copy K, tau, L into peltier_params.m and re-run s03.\n');

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
