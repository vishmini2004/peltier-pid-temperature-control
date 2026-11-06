%% S01_MODEL_ANALYSIS  Plant model, open-loop behaviour and reachable temperature range
% Report sections 5, 6 and 7 (open-loop part).
clear; clc; close all;
P = peltier_params();
M = peltier_plant(P);

fprintf('=== Peltier surface model (PROVISIONAL parameters) ===\n');
fprintf('K = %.2f degC/duty, tau = %.0f s, L = %.1f s, T_amb = %.1f degC\n', P.K, P.tau, P.L, P.Tamb);
disp('Supplied transfer function  Delta T_surface / Delta U :'); M.G
disp('With lumped dead time :'); M.Gdelay
disp('State-space model, x = T_surface - T_amb, inputs [u ; d_u] :'); M.ss
fprintf('Pole            : s = %.5f rad/s  (tau = %.0f s)\n', pole(M.G), P.tau);
fprintf('DC gain         : %.2f degC per unit duty\n', dcgain(M.G));
fprintf('Open-loop 2%% settling time : 4*tau = %.0f s\n', 4*P.tau);
fprintf('Bandwidth (-3 dB): %.4f rad/s = %.5f Hz\n', bandwidth(M.G), bandwidth(M.G)/(2*pi));

% ---- reachable steady-state range with a unidirectional driver (u in [0,1]) ----
Tmin = P.Tamb - P.K*P.umax;
fprintf('\nReachable steady-state range at T_amb = %.1f: %.1f .. %.1f degC\n', P.Tamb, Tmin, P.Tamb);
fprintf('Setpoints with 20%% duty headroom for disturbances: >= %.1f degC\n', P.Tamb - 0.8*P.K);

% ---- open-loop step responses in ABSOLUTE temperature -----------------------
t = (0:1:1200)';
fig = figure('Name', 'Open-loop step', 'Color', 'w');
hold on; grid on;
duties = [0.25 0.5 0.75 1.0];
for d = duties
    y = step(M.Gdelay*d, t);                % Delta T for a duty step of size d
    plot(t, P.Tamb + y, 'LineWidth', 1.4, 'DisplayName', sprintf('u = %.2f', d));
end
yline(Tmin, '--', sprintf('T_{min} = %.0f \\circC (u = 1)', Tmin), 'HandleVisibility', 'off');
xline(P.tau, ':', '\tau = 180 s', 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Surface temperature (\circC)');
title('Open-loop response to PWM duty steps (no controller)');
legend('Location', 'northeast');
save_fig(fig, 's01_open_loop_steps');

% ---- 63.2 % check (what the identification test looks for) ------------------
y1 = P.Tamb + step(M.G, t);
i63 = find(y1 <= P.Tamb - 0.632*P.K, 1);
fprintf('Open loop, u = 1: T reaches %.2f degC (63.2%%) at t = %d s; final %.2f degC\n', ...
        y1(i63), t(i63), y1(end));
i15 = find(y1 <= 15, 1);
fprintf('Fastest possible pull-down 25 -> 15 degC (u = 1 throughout): %d s\n', t(i15));

% ---- frequency response ---------------------------------------------------------
fig = figure('Name', 'Bode', 'Color', 'w');
bode(M.Gdelay, {1e-4, 1}); grid on;
title('Bode plot of -K e^{-Ls}/(\tau s + 1)');
save_fig(fig, 's01_bode');

% ---- pole-zero map ---------------------------------------------------------------
fig = figure('Name', 'Pole-zero', 'Color', 'w');
pzmap(M.G); grid on;
save_fig(fig, 's01_pzmap');
