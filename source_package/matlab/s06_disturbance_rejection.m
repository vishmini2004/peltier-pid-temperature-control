%% S06_DISTURBANCE_REJECTION  Physical heat-load disturbance vs the original model
% Report sections 7, 11 and Appendix B.
%   (a) heat load entering at the plant input (physical: heat must pass through
%       the plate's thermal capacitance before it changes the temperature)
%   (b) the SAME 3 degC disturbance applied as an instantaneous output step, as
%       in the originally supplied block diagram and plots - kept for comparison
%   (c) an ambient rise of +3 degC, which is mathematically the same as (a)
%       with d_u = Delta T_amb / K
clear; clc; close all;
P = peltier_params();
S = pid_designs(P);
names = {'PI_SIMC', 'PID_recommended', 'PI_conservative'};
step3 = 3;                                       % [degC] open-loop equivalent size

fig = figure('Name', 'Disturbance', 'Color', 'w', 'Position', [100 100 900 650]);
ax1 = subplot(2,1,1); hold on; grid on;
ax2 = subplot(2,1,2); hold on; grid on;
fprintf('%-17s %-14s %9s %8s %9s %6s\n', 'Design', 'Disturbance', 'peak', 'rec s', 'IAE', 'umax');
for i = 1:numel(names)
    g = S(strcmp({S.name}, names{i}));
    oa = simulate_loop(P, g);                                         % input load
    ob = simulate_loop(P, g, struct('dist', @(t) 0, 'out_dist', @(t) step3*(t >= P.t_load)));
    for c = {oa, 'input load'; ob, 'output step'}'
        o = c{1};
        m = dist_metrics(o.t, o.T, o.u, P.T_ref, P.t_load);
        fprintf('%-17s %-14s %9.3f %8.0f %9.1f %6.3f\n', names{i}, c{2}, m.peak, m.recovery, m.IAE, m.umax);
    end
    plot(ax1, oa.t, oa.T, 'LineWidth', 1.4, 'DisplayName', [strrep(names{i}, '_', ' ') ', heat load']);
    plot(ax2, ob.t, ob.T, 'LineWidth', 1.4, 'DisplayName', [strrep(names{i}, '_', ' ') ', output step']);
end
for ax = [ax1 ax2]
    yline(ax, P.T_ref, 'k--', 'HandleVisibility', 'off'); xlim(ax, [1100 2000]);
    ylabel(ax, 'Surface temperature (\circC)'); legend(ax, 'Location', 'northeast');
end
title(ax1, 'Physical heat load (enters through the thermal lag)');
title(ax2, 'Original model: instantaneous +3 \circC output step (not physical)');
xlabel(ax2, 'Time (s)');
save_fig(fig, 's06_disturbance');

% Ambient change check: +3 degC ambient == d_u = 3/K
fprintf('\nAmbient +%g degC is equivalent to a load of d_u = %.3f (same response as the input load).\n', ...
        step3, step3/P.K);
