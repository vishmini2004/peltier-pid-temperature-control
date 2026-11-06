function m = step_metrics(t, T, r, T0, t0, t1)
%STEP_METRICS  Setpoint-response metrics for a temperature step from T0 to r.
%   m = step_metrics(t, T, r, T0, t0, t1) evaluates the window t0 <= t < t1.
%   Works for cooling (r < T0) and warming (r > T0) steps by normalising
%   yn = (T - T0)/(r - T0), so 0 -> start, 1 -> setpoint.
%
%   tr   rise time 10 % -> 90 % of the change                      [s]
%   os   overshoot past the setpoint, % of the step size            [%]
%        (for a cooling step this is undershoot BELOW the setpoint)
%   ts   2 % settling time, measured from t0 (NaN = never settled)  [s]
%   ess  steady-state error r - mean(T over the last 60 s)          [degC]

if nargin < 6, t1 = inf; end
idx = find(t >= t0 & t < t1);
tt = t(idx); TT = T(idx);
step = r - T0;
yn = (TT - T0)/step;

i10 = find(yn >= 0.1, 1); i90 = find(yn >= 0.9, 1);
if isempty(i10) || isempty(i90), m.tr = NaN; else, m.tr = tt(i90) - tt(i10); end

m.os = max(0, (max(yn) - 1)*100);

band = 0.02*abs(step);
iout = find(abs(TT - r) > band, 1, 'last');
if isempty(iout),           m.ts = 0;
elseif iout == numel(TT),   m.ts = NaN;
else,                       m.ts = tt(iout) - t0;
end

tail = TT(tt >= tt(end) - 60);
m.ess = r - mean(tail);
end
