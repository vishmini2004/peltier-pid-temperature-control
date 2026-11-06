function M = peltier_plant(P)
%PELTIER_PLANT  Transfer-function and state-space forms of the Peltier surface model.
%   Requires the Control System Toolbox.
%
%   M.G       supplied model   Delta T_surface / Delta U = -K/(tau s + 1)
%   M.Gdelay  with lumped dead time L
%   M.Gpos    sign-absorbed plant used for loop analysis with the
%             reverse-acting (cooling) error e = T_meas - T_ref
%   M.Gload   Delta T_surface / d_u  (heat load in duty units, enters at the input)
%   M.ss      state space, inputs [u; d_u], output Delta T_surface

M.G      = tf(-P.K, [P.tau 1]);
M.Gdelay = tf(-P.K, [P.tau 1], 'InputDelay', P.L);
M.Gpos   = tf( P.K, [P.tau 1], 'InputDelay', P.L);
M.Gload  = tf( P.K, [P.tau 1]);

% x = T_surface - T_amb ;  dx/dt = -(1/tau) x - (K/tau) u + (K/tau) d_u
A = -1/P.tau;
B = [-P.K/P.tau, P.K/P.tau];
C = 1;
D = [0 0];
M.ss = ss(A, B, C, D, 'InputName', {'u','d_u'}, 'OutputName', {'dT_surface'}, ...
          'StateName', {'x'});
end
