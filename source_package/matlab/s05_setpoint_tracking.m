%% S05_SETPOINT_TRACKING  Setpoint feasibility and a multi-step tracking test
% Report sections 7 and 11. Part A shows why 5 and 10 degC cannot be held with
% the provisional parameters; Part B is the staircase test used in the demo.
clear; clc; close all;
P = peltier_params();
S = pid_designs(P);
rec = S(strcmp({S.name}, 'PID_recommended'));

%% Part A - feasibility: required steady-state duty u_ss = (T_amb - r)/K
refs = [5 10 13 15 18 20 22];
fprintf('%6s %8s %12s %10s %10s  %s\n', 'r', 'u_ss', 'u_ss+load', 'T@1150s', 'T@2350s', 'verdict');
fig = figure('Name', 'Feasibility', 'Color', 'w', 'Position', [100 100 900 650]);
ax1 = subplot(2,1,1); hold on; grid on;
ax2 = subplot(2,1,2); hold on; grid on;
for r = refs
    o = simulate_loop(P, rec, struct('ref', @(t) r));
    uss = (P.Tamb - r)/P.K; ussl = uss + P.d_load;
    if uss > 1,       v = 'INFEASIBLE: below T_amb - K';
    elseif ussl > 1,  v = 'MARGINAL: no headroom for the load';
    else,             v = 'feasible';
    end
    fprintf('%6.1f %8.3f %12.3f %10.2f %10.2f  %s\n', r, uss, ussl, ...
            mean(o.T(o.t >= 1100 & o.t < 1200)), mean(o.T(o.t >= 2300)), v);
    plot(ax1, o.t, o.T, 'LineWidth', 1.3, 'DisplayName', sprintf('r = %g \\circC', r));
    plot(ax2, o.t, o.u, 'LineWidth', 1.0, 'DisplayName', sprintf('r = %g \\circC', r));
end
yline(ax1, P.Tamb - P.K, 'r--', 'T_{amb} - K (u = 1 limit)', 'HandleVisibility', 'off');
ylabel(ax1, 'Surface temperature (\circC)'); legend(ax1, 'Location', 'eastoutside');
title(ax1, 'Setpoint feasibility with u \in [0,1], T_{amb} = 25 \circC, K = 15 \circC');
ylabel(ax2, 'Duty (0-1)'); xlabel(ax2, 'Time (s)'); legend(ax2, 'Location', 'eastoutside');
save_fig(fig, 's05_feasibility');

%% Part B - staircase: 20 -> 15 -> 18 -> 22 degC (cooling and passive warming)
ref = @(t) 20*(t < 800) + 15*(t >= 800 & t < 1600) + 18*(t >= 1600 & t < 2400) + 22*(t >= 2400);
o = simulate_loop(P, rec, struct('ref', ref, 'dist', @(t) 0, 't_end', 3200));
seg = [0 800 25 20; 800 1600 20 15; 1600 2400 15 18; 2400 3200 18 22];
fprintf('\n%10s %8s %8s %8s %8s\n', 'step', 'tr s', 'os %', 'ts s', 'ess');
for k = 1:size(seg,1)
    m = step_metrics(o.t, o.T, seg(k,4), seg(k,3), seg(k,1), seg(k,2));
    fprintf('%4g -> %-3g %8.0f %8.2f %8.0f %8.3f\n', seg(k,3), seg(k,4), m.tr, m.os, m.ts, m.ess);
end
fig = figure('Name', 'Staircase', 'Color', 'w', 'Position', [100 100 900 600]);
subplot(2,1,1); hold on; grid on;
plot(o.t, o.r, 'k--', 'DisplayName', 'Setpoint');
plot(o.t, o.T, 'LineWidth', 1.5, 'DisplayName', 'Surface temperature');
ylabel('Temperature (\circC)'); legend; title('Setpoint tracking with the recommended PID');
subplot(2,1,2); grid on;
plot(o.t, o.u, 'LineWidth', 1.2); ylabel('Duty (0-1)'); xlabel('Time (s)'); ylim([-0.05 1.05]);
save_fig(fig, 's05_staircase');
