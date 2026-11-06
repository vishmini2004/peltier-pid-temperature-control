function S = pid_designs(P)
%PID_DESIGNS  Candidate PID gain sets from standard FOPDT tuning rules.
%   Plant used for tuning:  |G| = K e^{-L s} / (tau s + 1)
%   The sign of the cooling plant is handled by the reverse-acting error
%   e = T_meas - T_ref, so all gains are positive.
%
%   Controller (parallel form):  C(s) = Kp + Ki/s + Kd*s/(Tf*s + 1)
%
%   Returns a struct array with fields name, Kp, Ki, Kd, Tf, method.

K = P.K; tau = P.tau; L = P.L; Tf = P.Tf;
S = struct('name', {}, 'Kp', {}, 'Ki', {}, 'Kd', {}, 'Tf', {}, 'method', {});

% 1) SIMC PI, conservative (lambda = 90 s)
lam = 90; Kc = tau/(K*(lam+L)); Ti = min(tau, 4*(lam+L));
S(end+1) = mk('PI_conservative', Kc, Kc/Ti, 0, Tf, 'SIMC PI, lambda = 90 s');

% 2) SIMC PI, lambda = 20 s (about 4L): the tuning target used in the report
lam = 20; Kc = tau/(K*(lam+L)); Ti = min(tau, 4*(lam+L));
S(end+1) = mk('PI_SIMC', Kc, Kc/Ti, 0, Tf, 'SIMC PI, lambda = 20 s');

% 3) Recommended PID: SIMC PI + derivative time Td = L/2 (dead-time compensation)
Td = L/2;
S(end+1) = mk('PID_recommended', Kc, Kc/Ti, Kc*Td, Tf, 'SIMC PI (lambda 20 s) + Td = L/2');

% 4) IMC-PID for FOPDT (Rivera, Morari & Skogestad 1986), lambda = 30 s
lam = 30; Kc = (tau + L/2)/(K*(lam + L/2)); Ti = tau + L/2; Td = tau*L/(2*tau + L);
S(end+1) = mk('PID_IMC_Rivera', Kc, Kc/Ti, Kc*Td, Tf, 'IMC-PID, lambda = 30 s');

% 5) Ziegler-Nichols open-loop (reaction-curve) PID - shown as a counter-example
Kc = 1.2*tau/(K*L); Ti = 2*L; Td = 0.5*L;
S(end+1) = mk('PID_ZN_reaction', Kc, Kc/Ti, Kc*Td, Tf, 'Ziegler-Nichols reaction curve');
end

function s = mk(name, Kp, Ki, Kd, Tf, method)
s = struct('name', name, 'Kp', Kp, 'Ki', Ki, 'Kd', Kd, 'Tf', Tf, 'method', method);
end
