function make_synthetic_data()
%MAKE_SYNTHETIC_DATA  Write clearly labelled SYNTHETIC logs so s08/s09 can be
%   exercised before the hardware exists. The "true" plant deliberately differs
%   from the provisional model (K = 13.2, tau = 205 s, L = 8 s, 0.05 degC noise)
%   so the identification has something to find.
%
%   THESE FILES ARE NOT MEASUREMENTS. Never put their numbers in the report.

here = fileparts(mfilename('fullpath'));
dd = fullfile(here, '..', 'data');
if ~exist(dd, 'dir'), mkdir(dd); end

Ptrue = peltier_params();
Ptrue.K = 13.2; Ptrue.tau = 205; Ptrue.L = 8; Ptrue.noise_sd = 0.05;

% ---- open-loop step: duty 0 for 120 s, then 0.6 for 1200 s -------------------
o = simulate_loop(Ptrue, [], struct('mode', 'open', 'u_ff', @(t) 0.6*(t >= 120), ...
                                    'dist', @(t) 0, 't_end', 1320, 'seed', 7));
write_log(fullfile(dd, 'SYNTHETIC_openloop_step.csv'), o, 'OPEN');

% ---- closed-loop run: staircase with a heater pulse, recommended gains ------
S = pid_designs(peltier_params());
g = S(strcmp({S.name}, 'PID_recommended'));
ref  = @(t) 20*(t < 900) + 15*(t >= 900 & t < 2100) + 18*(t >= 2100);
heater = @(t) double(t >= 1500 & t < 2100);
o = simulate_loop(Ptrue, g, struct('ref', ref, 'dist', @(t) 0.2*heater(t), 't_end', 2700, 'seed', 8));
o.d = heater(o.t);
write_log(fullfile(dd, 'SYNTHETIC_closedloop_run.csv'), o, 'AUTO');
end

function write_log(f, o, mode)
fid = fopen(f, 'w');
fprintf(fid, 't_s,T_surf,T_hot,T_ref,duty,load,mode\n');
for k = 1:numel(o.t)
    fprintf(fid, '%.1f,%.4f,%.2f,%.2f,%.4f,%d,%s\n', o.t(k), o.Tm(k), 30 + 8*o.u(k), o.r(k), ...
            o.u(k), round(o.d(k) > 0), mode);
end
fclose(fid);
end
