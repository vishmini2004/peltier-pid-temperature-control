%% S02_BEFORE_AFTER  System response before and after applying the controller
% Assignment requirement 3: "demonstrate the response of the system before and
% after applying the controller". Four strategies, same test:
%   setpoint 15 degC from 25 degC, heat load d_u = 0.2 (3 degC equivalent) at 1200 s.
%   1. Open loop (no feedback): fixed duty u = (T_amb - T_ref)/K computed from the model
%   2. Proportional only (no integral action)
%   3. On/off thermostat with +/-0.5 degC hysteresis
%   4. Recommended PID
clear; clc; close all;
P = peltier_params();
S = pid_designs(P);
rec = S(strcmp({S.name}, 'PID_recommended'));

cases = {
  'Open loop (fixed duty)',  struct('mode', 'open'),  rec
  'P only',                  struct(),                setfield(setfield(rec, 'Ki', 0), 'Kd', 0) %#ok<SFLD>
  'On/off, \pm0.5 \circC',   struct('mode', 'onoff'), rec
  'PID (recommended)',       struct(),                rec };

fig = figure('Name', 'Before vs after', 'Color', 'w', 'Position', [100 100 900 650]);
ax1 = subplot(2,1,1); hold on; grid on;
ax2 = subplot(2,1,2); hold on; grid on;
fprintf('%-26s %10s %10s %10s %10s\n', 'Strategy', 'T@1150s', 'T end', 'p-p', 'std(u)');
for i = 1:size(cases,1)
    o = simulate_loop(P, cases{i,3}, cases{i,2});
    plot(ax1, o.t, o.T, 'LineWidth', 1.4, 'DisplayName', cases{i,1});
    plot(ax2, o.t, o.u, 'LineWidth', 1.0, 'DisplayName', cases{i,1});
    pre  = mean(o.T(o.t >= 1100 & o.t < 1200));
    last = o.T(o.t > o.t(end) - 150);       % last full on/off cycle (period 150 s)
    two  = o.T(o.t > o.t(end) - 300);       % last two cycles, for peak-to-peak
    fprintf('%-26s %10.2f %10.2f %10.2f %10.3f\n', cases{i,1}, pre, mean(last), ...
            max(two) - min(two), std(o.u(o.t >= 1500)));
end
yline(ax1, P.T_ref, 'k--', 'Setpoint', 'HandleVisibility', 'off');
xline(ax1, P.t_load, ':', 'Heat load on', 'HandleVisibility', 'off');
ylabel(ax1, 'Surface temperature (\circC)'); legend(ax1, 'Location', 'northeast');
title(ax1, 'Before and after control: setpoint 15 \circC, heat load at 1200 s');
ylabel(ax2, 'Duty (0-1)'); xlabel(ax2, 'Time (s)'); ylim(ax2, [-0.05 1.05]);
save_fig(fig, 's02_before_after');
