function P = peltier_params()
%PELTIER_PARAMS  Single source of truth for every model, sensor and test parameter.
%
%   Every value carries a status tag:
%     PROVISIONAL - assumed for the preliminary simulation; REPLACE with the
%                   value identified from your own hardware (run s08_identify_plant)
%     DESIGN      - chosen by the designer
%     DATASHEET   - taken from a component datasheet
%
%   Model (deviation form, x = T_surface - T_amb):
%       tau * dx/dt = -x - K*(u - d_u)          u   = normalised PWM duty, 0..1
%                                               d_u = heat load expressed in duty units
%   i.e. Delta T_surface(s) / Delta U(s) = -K / (tau*s + 1)   (supplied model)

% ---------------- plant (PROVISIONAL until identified) ----------------------
P.Tamb  = 25;      % [degC]               PROVISIONAL  lab ambient on the test day
P.K     = 15;      % [degC / unit duty]   PROVISIONAL  steady-state cooling at u = 1
P.tau   = 180;     % [s]                  PROVISIONAL  surface thermal time constant
P.L     = 5;       % [s]                  PROVISIONAL  lumped dead time (sensor lag,
                   %                                   contact resistance, conversion)

% ---------------- sensor / controller ---------------------------------------
P.Ts    = 1.0;     % [s]      DESIGN     controller sample period (firmware loop)
P.q     = 0.0625;  % [degC]   DATASHEET  DS18B20 12-bit resolution
P.noise_sd = 0;    % [degC]   DESIGN     sensor noise for noise studies (e.g. 0.05)
P.umin  = 0;       % [-]      DESIGN     unidirectional MOSFET driver: cooling only
P.umax  = 1;       % [-]
P.Tf    = 2.0;     % [s]      DESIGN     derivative filter time constant (>= Ts)

% ---------------- simulation ------------------------------------------------
P.dt     = 0.1;    % [s]  plant integration step (exact ZOH discretisation)
P.log_dt = 1.0;    % [s]  logging interval of the returned signals
P.t_end  = 2400;   % [s]

% ---------------- standard test scenario ------------------------------------
P.T_ref  = 15;     % [degC]  standard setpoint (feasible: needs u = 0.667)
P.t_load = 1200;   % [s]     load step time
P.d_load = 0.2;    % [duty]  heat load; 0.2*K = 3 degC open-loop rise

end
