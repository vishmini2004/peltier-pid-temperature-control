%% peltier_surface_pid_sweep.m
% PID test of the Peltier SURFACE temperature model (not air) for 5, 10, 15, 20 degC.
% Pure MATLAB (needs Control System Toolbox for tf/pidtune only). No Simulink required.
clc; clear; close all;

%% 1. Plant: Peltier surface temperature - replace with your measured values
K     = 15;      % degC of cooling per 100 % duty
tau   = 180;     % s
L     = 0;       % s transport delay (0 = none)
Tamb  = 25;      % degC (surface temperature at duty = 0)
Tsets = [5 10 15 20];

tStep = 10;      % s, setpoint applied
tDist = 1200;    % s, disturbance applied (heat load)
dDist = 3;       % degC rise of surface temperature caused by the disturbance
tEnd  = 2400;    % s
dt    = 0.1;     % s

%% 2. PID design (on the cooling model, positive gain)
Gc   = tf(K,[tau 1],'InputDelay',L);
Cpi  = pidtune(Gc,'PI');
Cpid = pidtune(Gc,'PID');
Npid = 100; if Cpid.Tf > 0, Npid = 1/Cpid.Tf; end

ctrl(1) = struct('name','PI' ,'Kp',Cpi.Kp ,'Ki',Cpi.Ki ,'Kd',0      ,'N',100);
ctrl(2) = struct('name','PID','Kp',Cpid.Kp,'Ki',Cpid.Ki,'Kd',Cpid.Kd,'N',Npid);
fprintf('PI : Kp = %.4f, Ki = %.5f\n', Cpi.Kp, Cpi.Ki);
fprintf('PID: Kp = %.4f, Ki = %.5f, Kd = %.4f, N = %.2f\n', Cpid.Kp, Cpid.Ki, Cpid.Kd, Npid);
fprintf('Lowest reachable surface temperature = Tamb - K = %.1f degC\n\n', Tamb-K);

%% 3. Run all setpoints, PI and PID
results = table();
S = cell(numel(Tsets),2);
for i = 1:numel(Tsets)
    Tset  = Tsets(i);
    ref   = Tamb - Tset;               % cooling needed (degC)
    dutyN = ref/K;                     % steady-state duty needed
    if     dutyN <= 0.95, status = 'OK';
    elseif dutyN <= 1.00, status = 'Marginal';
    else,                 status = 'Unreachable';
    end
    for c = 1:2
        [t,T,u] = simPeltier(Tset,ctrl(c),K,tau,L,Tamb,tStep,tDist,tEnd,dDist,dt);
        S{i,c} = struct('t',t,'T',T,'u',u);
        m = calcMetrics(t,T,Tset,Tamb,K,tStep,tDist);
        results = [results; table(Tset,{ctrl(c).name},{status},dutyN*100,m.Rise,m.Settle,m.OS, ...
            m.Ess,m.DistPeak,m.Recovery,max(u)*100,mean(T(t>=tEnd-60)), ...
            'VariableNames',{'Tset_C','Ctrl','Status','DutyNeeded_pct','Rise_s','Settle_s','OS_pct', ...
            'Ess_C','DistPeak_C','Recovery_s','MaxDuty_pct','T_final_C'})]; %#ok<AGROW>
    end
end
disp(results);
writetable(results,'surface_pid_results.csv');

%% 4. Figures
colPI = [0 0.447 0.741]; colPID = [0.85 0.325 0.098];

figure('Name','Surface temperature','Position',[80 80 1000 700]);
for i = 1:numel(Tsets)
    subplot(2,2,i); hold on; grid on;
    plot(S{i,1}.t,S{i,1}.T,'Color',colPI ,'LineWidth',1.4);
    plot(S{i,2}.t,S{i,2}.T,'Color',colPID,'LineWidth',1.4);
    yline(Tsets(i),'--k'); xline(tDist,':m');
    title(sprintf('Peltier surface temperature, setpoint = %d \\circC',Tsets(i)));
    xlabel('Time (s)'); ylabel('Surface temperature (\circC)');
    legend('PI','PID','Setpoint','Disturbance','Location','best');
end
saveas(gcf,'surface_temp_PI_vs_PID.png');

figure('Name','Duty','Position',[120 120 1000 700]);
for i = 1:numel(Tsets)
    subplot(2,2,i); hold on; grid on;
    plot(S{i,1}.t,S{i,1}.u,'Color',colPI ,'LineWidth',1.4);
    plot(S{i,2}.t,S{i,2}.u,'Color',colPID,'LineWidth',1.4);
    xline(tDist,':m'); ylim([-0.05 1.05]);
    title(sprintf('Duty (control effort), setpoint = %d \\circC',Tsets(i)));
    xlabel('Time (s)'); ylabel('Duty (0-1)'); legend('PI','PID','Location','best');
end
saveas(gcf,'surface_duty_PI_vs_PID.png');

figure('Name','All setpoints (PID)'); hold on; grid on;
cols = lines(numel(Tsets));
for i = 1:numel(Tsets)
    plot(S{i,2}.t,S{i,2}.T,'Color',cols(i,:),'LineWidth',1.4);
    yline(Tsets(i),':','Color',cols(i,:),'HandleVisibility','off');
end
xline(tDist,':m','HandleVisibility','off');
legend(arrayfun(@(x) sprintf('%d \\circC',x),Tsets,'UniformOutput',false),'Location','northeast');
xlabel('Time (s)'); ylabel('Surface temperature (\circC)');
title('Peltier surface temperature - PID, all setpoints');
saveas(gcf,'surface_all_setpoints_PID.png');

%% ===================== local functions =====================
function [t,T,u] = simPeltier(Tset,c,K,tau,L,Tamb,tStep,tDist,tEnd,dDist,dt)
    t  = (0:dt:tEnd)';  n = numel(t);
    nd = max(round(L/dt),1);  buf = zeros(nd,1);  bi = 1;
    a  = exp(-dt/tau);                    % exact first-order discretisation
    af = 1 - exp(-c.N*dt);                % derivative filter
    x = 0; integ = 0; xf = 0;
    T = zeros(n,1); u = zeros(n,1);
    for k = 1:n
        if L > 0, xd = buf(bi); else, xd = x; end
        d  = dDist*(t(k) >= tDist);
        Tm = Tamb - xd + d;               % measured surface temperature
        if t(k) >= tStep, e = Tm - Tset; else, e = 0; end   % e > 0 -> too hot -> cool more
        uu = c.Kp*e + integ + c.Kd*c.N*(e - xf);
        us = min(max(uu,0),1);            % duty limits 0..1 (Peltier cannot heat here)
        if us == uu || (uu > 1 && e < 0) || (uu < 0 && e > 0)   % clamping anti-windup
            integ = integ + c.Ki*e*dt;
        end
        xf = xf + af*(e - xf);
        T(k) = Tm; u(k) = us;
        x = a*x + (1-a)*K*us;
        if L > 0, buf(bi) = x; bi = mod(bi,nd) + 1; end
    end
end

function m = calcMetrics(t,T,Tset,Tamb,K,tStep,tDist)
    m = struct('Rise',NaN,'Settle',NaN,'OS',NaN,'Ess',NaN,'DistPeak',NaN,'Recovery',NaN);
    ref = Tamb - Tset;
    pre = t >= tStep & t < tDist;
    y   = Tamb - T(pre);  tt = t(pre) - tStep;
    m.Ess = Tset - mean(T(t >= tDist-60 & t < tDist));
    post = t >= tDist;  Tp = T(post);  tp = t(post);
    m.DistPeak = max(Tp) - Tset;
    out = find(abs(Tp - Tset) > 0.5, 1, 'last');       % last time outside +-0.5 degC
    if isempty(out), m.Recovery = 0; elseif out < numel(Tp), m.Recovery = tp(out) - tDist; end
    if ref <= K                                         % setpoint physically reachable
        i10 = find(y >= 0.1*ref,1);  i90 = find(y >= 0.9*ref,1);
        if ~isempty(i10) && ~isempty(i90), m.Rise = tt(i90) - tt(i10); end
        m.OS = max(0,(max(y) - ref)/ref*100);
        o = find(abs(y - ref) > 0.02*ref,1,'last');     % 2 % settling band
        if isempty(o), m.Settle = 0; elseif o < numel(y), m.Settle = tt(o); end
    end
end
