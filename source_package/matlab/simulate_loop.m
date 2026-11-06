function out = simulate_loop(P, g, opts)
%SIMULATE_LOOP  Closed-loop Peltier simulation with the effects a real loop has:
%   actuator saturation, clamping anti-windup, dead time, sampled control,
%   sensor quantisation and optional sensor noise. No toolbox needed.
%
%   out = simulate_loop(P, g, opts)
%     P     parameter struct from peltier_params
%     g     gains struct with fields Kp, Ki, Kd, Tf (see pid_designs)
%     opts  (all optional)
%       .mode      'pid' (default) | 'open' | 'onoff'
%       .ref       @(t) setpoint [degC]                (default constant P.T_ref)
%       .dist      @(t) heat load in duty units, enters at the plant input
%                  (default: P.d_load step at P.t_load)
%       .out_dist  @(t) [degC] added directly to the surface temperature.
%                  This reproduces the ORIGINAL model's disturbance and is kept
%                  only for comparison - it is not physical (see report, App. B)
%       .u_ff      @(t) duty for 'open' mode             (default feedforward)
%       .band      on/off half-band [degC]               (default 0.5)
%       .t_end     [s]                                   (default P.t_end)
%       .seed      noise seed                            (default 1)
%
%   Control law (reverse acting, because the plant gain is negative):
%       e = T_meas - T_ref             positive error -> too warm -> more cooling
%       u = sat( Kp*e + I + D )        I clamped while saturated in the same direction
%       D = derivative of the MEASUREMENT through a first-order filter Tf
%   This is exactly the law implemented in firmware/peltier_pid/peltier_pid.ino.
%
%   Mirrors verification/verify_sim.py line for line (used to cross-check numbers).

if nargin < 3, opts = struct(); end
mode   = getopt(opts, 'mode', 'pid');
reff   = getopt(opts, 'ref',  @(t) P.T_ref);
distf  = getopt(opts, 'dist', @(t) P.d_load*(t >= P.t_load));
odistf = getopt(opts, 'out_dist', @(t) 0);
uff    = getopt(opts, 'u_ff', @(t) (P.Tamb - reff(t))/P.K);
band   = getopt(opts, 'band', 0.5);
t_end  = getopt(opts, 't_end', P.t_end);
seed   = getopt(opts, 'seed', 1);

a      = exp(-P.dt/P.tau);              % exact discretisation of the first-order lag
nd     = round(P.L/P.dt);               % dead time in plant steps
n      = round(t_end/P.dt) + 1;
every  = round(P.Ts/P.dt);              % controller runs every 'every' plant steps
logev  = round(P.log_dt/P.dt);

x = 0;                                  % deviation from ambient, starts at T_amb
buf = P.Tamb*ones(nd+1, 1); head = 1;   % circular buffer = transport delay
I = 0; D = 0; y_prev = NaN; u = 0; relay = 0;
rs = RandStream('mt19937ar', 'Seed', seed);

nlog = floor((n-1)/logev) + 1;
out.t = zeros(nlog,1); out.T = out.t; out.Tm = out.t; out.r = out.t; out.u = out.t;
out.d = out.t; out.e = out.t; out.P = out.t; out.I = out.t; out.D = out.t;
j = 0;

for k = 0:n-1
    t = k*P.dt;
    T = P.Tamb + x + odistf(t);                 % actual surface temperature

    % transport delay: write newest sample, read the one nd steps old
    buf(head) = T;
    head = mod(head, nd+1) + 1;
    Td = buf(head);

    Tm = Td;
    if P.noise_sd > 0, Tm = Tm + P.noise_sd*randn(rs); end
    if P.q > 0,        Tm = round(Tm/P.q)*P.q;     end   % sensor quantisation
    r = reff(t);

    if mod(k, every) == 0                       % sampled controller
        e = Tm - r;
        switch mode
            case 'open'
                u = min(P.umax, max(P.umin, uff(t)));
            case 'onoff'
                if e >  band, relay = 1; end
                if e < -band, relay = 0; end
                u = relay;
            otherwise
                if isnan(y_prev), y_prev = Tm; end
                D = (g.Tf/(g.Tf + P.Ts))*D + (g.Kd/(g.Tf + P.Ts))*(Tm - y_prev);
                y_prev = Tm;
                Pt = g.Kp*e;
                I_new = I + g.Ki*P.Ts*e;
                v = Pt + I_new + D;
                if (v > P.umax && e > 0) || (v < P.umin && e < 0)
                    I_new = I;                  % clamping anti-windup
                end
                I = I_new;
                u = min(P.umax, max(P.umin, Pt + I + D));
        end
    end
    du = distf(t);

    if mod(k, logev) == 0
        j = j + 1;
        out.t(j) = t; out.T(j) = T; out.Tm(j) = Tm; out.r(j) = r; out.u(j) = u;
        out.d(j) = du; out.e(j) = Tm - r;
        if strcmp(mode, 'pid'), out.P(j) = g.Kp*(Tm - r); out.I(j) = I; out.D(j) = D; end
    end

    x = a*x + (1 - a)*(-P.K*(u - du));          % plant update (ZOH on u and d_u)
end
end

function v = getopt(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
