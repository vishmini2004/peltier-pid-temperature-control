%% S07_ROBUSTNESS  Recommended PID against plant-parameter uncertainty
% The provisional K, tau and L may be wrong by tens of percent until they are
% measured; this checks the design still works. Report sections 8 and 13.
clear; clc; close all;
P0 = peltier_params();
S = pid_designs(P0);
rec = S(strcmp({S.name}, 'PID_recommended'));     % gains fixed at the nominal design

cases = { 'nominal', struct();  'K x0.7', struct('K', 0.7*P0.K);  'K x1.3', struct('K', 1.3*P0.K);
          'tau x0.7', struct('tau', 0.7*P0.tau);  'tau x1.3', struct('tau', 1.3*P0.tau);
          'L = 10 s', struct('L', 10);  'L = 20 s', struct('L', 20);
          'K x1.3, L = 20 s', struct('K', 1.3*P0.K, 'L', 20) };
fprintf('%-18s %6s | %6s %6s | %6s %6s | %7s %6s\n', 'Plant', 'r', 'PM', 'GM dB', 'os %', 'ts s', 'pk degC', 'rec s');
fig = figure('Name', 'Robustness', 'Color', 'w'); hold on; grid on;
for i = 1:size(cases,1)
    P = P0; f = fieldnames(cases{i,2});
    for k = 1:numel(f), P.(f{k}) = cases{i,2}.(f{k}); end
    r = max(15, P.Tamb - 0.8*P.K);                   % keep the setpoint reachable when K shrinks
    o = simulate_loop(P, rec, struct('ref', @(t) r));
    mg = loop_margins(P, rec);
    ms = step_metrics(o.t, o.T, r, P.Tamb, 0, P.t_load);
    md = dist_metrics(o.t, o.T, o.u, r, P.t_load);
    fprintf('%-18s %6.1f | %6.1f %6.1f | %6.2f %6.0f | %7.3f %6.0f\n', cases{i,1}, r, mg.PM, mg.GM_dB, ...
            ms.os, ms.ts, md.peak, md.recovery);
    plot(o.t, o.T - r, 'LineWidth', 1.2, 'DisplayName', cases{i,1});
end
xlabel('Time (s)'); ylabel('T - r (\circC)'); ylim([-1 3]); legend;
title('Recommended PID with perturbed plant parameters (error from setpoint)');
save_fig(fig, 's07_robustness');
