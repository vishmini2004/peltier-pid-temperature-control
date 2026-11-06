%% S03_PID_DESIGN  Candidate tunings: gains, stability margins, time-domain metrics
% Report section 8. Compares SIMC PI, recommended PID, IMC-PID and Ziegler-Nichols.
clear; clc; close all;
P = peltier_params();
S = pid_designs(P);

fprintf('%-17s %8s %9s %7s | %6s %6s %7s | %7s %7s | %7s %6s %7s\n', 'Design', 'Kp', 'Ki', 'Kd', ...
        'PM', 'GM dB', 'Ldm s', 'tr s', 'ts s', 'pk degC', 'rec s', 'std u');
fig = figure('Name', 'Design comparison', 'Color', 'w', 'Position', [100 100 900 650]);
ax1 = subplot(2,1,1); hold on; grid on;
ax2 = subplot(2,1,2); hold on; grid on;
Pn = P; Pn.noise_sd = 0.05;                 % noise study: 0.05 degC rms sensor noise
for i = 1:numel(S)
    g = S(i);
    o  = simulate_loop(P,  g);
    on = simulate_loop(Pn, g);
    ms = step_metrics(o.t, o.T, P.T_ref, P.Tamb, 0, P.t_load);
    md = dist_metrics(o.t, o.T, o.u, P.T_ref, P.t_load);
    mg = loop_margins(P, g);
    su = std(on.u(on.t >= 800 & on.t < 1190));
    fprintf('%-17s %8.4f %9.6f %7.4f | %6.1f %6.1f %7.1f | %7.0f %7.0f | %7.3f %6.0f %7.4f\n', ...
            g.name, g.Kp, g.Ki, g.Kd, mg.PM, mg.GM_dB, mg.delay_margin, ms.tr, ms.ts, ...
            md.peak, md.recovery, su);
    plot(ax1, o.t, o.T, 'LineWidth', 1.3, 'DisplayName', strrep(g.name, '_', ' '));
    plot(ax2, o.t, o.u, 'LineWidth', 1.0, 'DisplayName', strrep(g.name, '_', ' '));
end
yline(ax1, P.T_ref, 'k--', 'HandleVisibility', 'off');
ylabel(ax1, 'Surface temperature (\circC)'); legend(ax1); title(ax1, 'Tuning comparison');
ylabel(ax2, 'Duty (0-1)'); xlabel(ax2, 'Time (s)');
save_fig(fig, 's03_design_comparison');

% ---- cross-check against MATLAB's own tuner (if available) ---------------------
if exist('pidtune', 'file')
    M = peltier_plant(P);
    wc = 1/(20 + P.L);                       % same target as SIMC lambda = 20 s
    [Cp, info] = pidtune(M.Gpos, 'PIDF', wc);
    fprintf('\npidtune cross-check (wc = %.4f rad/s): Kp = %.4f, Ki = %.6f, Kd = %.4f, PM = %.1f deg\n', ...
            wc, Cp.Kp, Cp.Ki, Cp.Kd, info.PhaseMargin);
end

% ---- loop Bode and closed-loop step for the recommended design ---------------
rec = S(strcmp({S.name}, 'PID_recommended'));
M = peltier_plant(P);
C = pid(rec.Kp, rec.Ki, rec.Kd, rec.Tf);
fig = figure('Name', 'Loop margins', 'Color', 'w');
margin(C*M.Gpos); grid on;
save_fig(fig, 's03_loop_margins');
