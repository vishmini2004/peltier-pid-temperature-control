function m = dist_metrics(t, T, u, r, td, t1, band)
%DIST_METRICS  Load-disturbance metrics after a disturbance applied at td.
%   peak      largest temperature rise above the setpoint          [degC]
%   recovery  time from td until T stays inside r +/- band         [s]
%   IAE       integral of |T - r| over the window                  [degC s]
%   umax      largest duty used (1.0 = actuator limit reached)      [-]

if nargin < 6 || isempty(t1), t1 = inf; end
if nargin < 7, band = 0.2; end
idx = find(t >= td & t < t1);
tt = t(idx); TT = T(idx);
m.peak = max(TT - r);
iout = find(abs(TT - r) > band, 1, 'last');
if isempty(iout), m.recovery = 0; else, m.recovery = tt(iout) - td; end
m.IAE  = trapz(tt, abs(TT - r));
m.umax = max(u(idx));
end
