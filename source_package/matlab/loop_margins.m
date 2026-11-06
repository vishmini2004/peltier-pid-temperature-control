function m = loop_margins(P, g)
%LOOP_MARGINS  Gain/phase margins of the cooling loop (Control System Toolbox).
%   Loop transfer L(s) = C(s) * K e^{-L s}/(tau s + 1). The minus sign of the
%   plant is cancelled by the reverse-acting error e = T_meas - T_ref, so the
%   loop is analysed with the positive plant M.Gpos.
%
%   Note: the 1 s sampling adds about Ts/2 = 0.5 s of extra effective delay,
%   which these continuous-time margins do not include.

M = peltier_plant(P);
C = pid(g.Kp, g.Ki, g.Kd, g.Tf);          % Kp + Ki/s + Kd s/(Tf s + 1)
[Gm, Pm, Wcg, Wcp] = margin(C*M.Gpos);
m.GM_dB = 20*log10(Gm);
m.PM    = Pm;
m.w180  = Wcg;                             % phase-crossover frequency [rad/s]
m.wc    = Wcp;                             % gain-crossover frequency  [rad/s]
m.delay_margin = deg2rad(Pm)/Wcp;          % extra dead time tolerated [s]
end
