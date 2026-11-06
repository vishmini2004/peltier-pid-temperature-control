%% S04_GAIN_SWEEP  Effect of increasing / decreasing Kp, Ki and Kd
% Report section 8 and discussion questions 2-3. One gain varied at a time
% around the recommended set; every run uses the standard test.
clear; clc; close all;
P = peltier_params();
S = pid_designs(P);
rec = S(strcmp({S.name}, 'PID_recommended'));
Pn = P; Pn.noise_sd = 0.05;

sweeps = { 'Kp', [0.5 1 2 4];  'Ki', [0.25 1 4 10];  'Kd', [0 1 4] };
fig = figure('Name', 'Gain sweep', 'Color', 'w', 'Position', [50 50 1100 800]);
fprintf('%-10s %8s %7s %7s %7s | %7s %7s | %6s %6s | %7s\n', 'Change', 'gain', 'os %', 'tr s', 'ts s', ...
        'pk degC', 'rec s', 'PM', 'GM dB', 'std u');
for s = 1:size(sweeps,1)
    name = sweeps{s,1};
    ax1 = subplot(3, 2, 2*s-1); hold on; grid on;
    ax2 = subplot(3, 2, 2*s);   hold on; grid on;
    for f = sweeps{s,2}
        g = rec; g.(name) = rec.(name)*f;
        o  = simulate_loop(P,  g);
        on = simulate_loop(Pn, g);
        ms = step_metrics(o.t, o.T, P.T_ref, P.Tamb, 0, P.t_load);
        md = dist_metrics(o.t, o.T, o.u, P.T_ref, P.t_load);
        mg = loop_margins(P, g);
        fprintf('%-10s %8.4f %7.2f %7.0f %7.0f | %7.3f %7.0f | %6.1f %6.1f | %7.4f\n', ...
                sprintf('%s x%g', name, f), g.(name), ms.os, ms.tr, ms.ts, md.peak, md.recovery, ...
                mg.PM, mg.GM_dB, std(on.u(on.t >= 800 & on.t < 1190)));
        lbl = sprintf('%s \\times %g', name, f);
        plot(ax1, o.t, o.T, 'LineWidth', 1.2, 'DisplayName', lbl);
        plot(ax2, on.t, on.u, 'LineWidth', 0.8, 'DisplayName', lbl);
    end
    yline(ax1, P.T_ref, 'k--', 'HandleVisibility', 'off');
    ylabel(ax1, 'T (\circC)'); title(ax1, ['Varying ' name]); legend(ax1, 'Location', 'northeast');
    ylabel(ax2, 'Duty (with 0.05 \circC noise)'); title(ax2, ['Control effort, varying ' name]);
    if s == 3, xlabel(ax1, 'Time (s)'); xlabel(ax2, 'Time (s)'); end
end
save_fig(fig, 's04_gain_sweep');
