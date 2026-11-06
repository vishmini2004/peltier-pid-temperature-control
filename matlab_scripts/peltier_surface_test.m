%% peltier_surface_test.m
% Peltier surface-temperature test: model results + measured results in one run.
% Measured files (optional): surface_20.csv, surface_40.csv, ... with columns
%   time_s, duty (percent), temp_C    (made by peltier_open_loop.ino)
clc; clear; close all;

%% 1. Parameters (illustrative - replace with measured values, or run with CSV files)
K_surface   = 15;                    % degC drop per 100 % duty
tau_surface = 180;                   % s
Tamb        = 25;                    % degC
duties      = [0.2 0.4 0.6 0.8 1.0]; % duty fractions to compare
t           = (0:1:6*tau_surface)';

s = tf('s');
G_surface = -K_surface/(tau_surface*s + 1);

disp('Peltier Surface Transfer Function:');
G_surface
fprintf('DC gain       = %.2f degC per 100%% duty\n', dcgain(G_surface));
fprintf('Pole          = %.5f rad/s\n', pole(G_surface));
fprintf('Time constant = %.1f s\n', -1/pole(G_surface));

%% 2. Model step responses for all duty levels
figure('Name','Model step responses'); hold on; grid on;
leg = strings(1,numel(duties));
for i = 1:numel(duties)
    y = step(duties(i)*G_surface, t);
    plot(t, Tamb + y, 'LineWidth', 1.5);
    leg(i) = sprintf('%d %% duty', round(duties(i)*100));
end
yline(Tamb,'--k','Ambient');
xline(tau_surface,':','\tau');
legend(leg,'Location','southeast');
title('Peltier surface temperature - model step response');
xlabel('Time (s)'); ylabel('Surface temperature (\circC)');
saveas(gcf,'surface_model_steps.png');

%% 3. Model metrics table (first-order system)
rise1090 = tau_surface*log(9);       % 10-90 % rise time
settle2  = tau_surface*log(50);      % 2 % settling time
dT       = -K_surface*duties(:);
Tfinal   = Tamb + dT;
model_tbl = table(duties(:)*100, dT, Tfinal, ...
    repmat(tau_surface,numel(duties),1), repmat(rise1090,numel(duties),1), ...
    repmat(settle2,numel(duties),1), ...
    'VariableNames',{'Duty_pct','DeltaT_C','T_final_C','Tau_s','Rise10_90_s','Settle2pct_s'});
disp('--- Model metrics ---'); disp(model_tbl);
writetable(model_tbl,'surface_model_metrics.csv');

%% 4. Static characteristic and frequency response
figure('Name','Static characteristic + Bode');
subplot(1,2,1); plot(duties*100, Tfinal, '-o','LineWidth',1.5); grid on; hold on;
xlabel('Duty (%)'); ylabel('Steady-state surface temp (\circC)'); title('Static characteristic (model)');
subplot(1,2,2); bode(G_surface); grid on; title('Bode diagram');
saveas(gcf,'surface_static_bode.png');

%% 5. Measured results (runs only if CSV files exist)
files = dir('surface_*.csv');
files = files(~strcmp({files.name},'surface_model_metrics.csv'));
if isempty(files)
    fprintf('\nNo measured files (surface_20.csv ...) found -> model results only.\n');
else
    R = table(); n = numel(files);
    figure('Name','Measured vs fitted model','Position',[100 100 1000 650]);
    for i = 1:n
        d    = readtable(files(i).name);
        tm   = d.time_s - d.time_s(1);
        Tm   = d.temp_C;
        duty = max(d.duty)/100;
        T0   = mean(Tm(1:min(5,numel(Tm))));
        Tend = mean(Tm(max(1,end-19):end));

        % initial guess for tau: time to reach 63.2 % of the total change
        frac = (Tm - T0)/(Tend - T0);
        k63  = find(frac >= 0.632, 1);
        if isempty(k63), tau0 = max(tm)/4; else, tau0 = tm(k63); end

        % fit  T = T0 + dT*(1 - exp(-(t-L)/tau))   for t >= L
        model = @(p) T0 + p(1)*(1 - exp(-max(tm - abs(p(3)),0)/abs(p(2))));
        cost  = @(p) sum((model(p) - Tm).^2);
        p = fminsearch(cost,[Tend-T0, tau0, 1], ...
            optimset('MaxFunEvals',5000,'TolX',1e-6,'TolFun',1e-9,'Display','off'));

        dTm = p(1); taum = abs(p(2)); Lm = abs(p(3)); Km = dTm/duty;
        Tfit = model(p);
        SSres = sum((Tm-Tfit).^2); SStot = sum((Tm-mean(Tm)).^2);
        R2 = 1 - SSres/SStot; rmse = sqrt(mean((Tm-Tfit).^2));

        % model (nominal parameters) for comparison
        Tnom = T0 - K_surface*duty*(1 - exp(-tm/tau_surface));

        subplot(ceil(n/2),2,i); hold on; grid on;
        plot(tm,Tm,'r','LineWidth',1.2); plot(tm,Tfit,'b--','LineWidth',1.4);
        plot(tm,Tnom,'k:','LineWidth',1.2);
        title(sprintf('%d %% duty:  K=%.1f, \\tau=%.0f s, R^2=%.3f',round(duty*100),Km,taum,R2));
        xlabel('Time (s)'); ylabel('Surface temp (\circC)');
        legend('Measured','Fitted','Nominal model','Location','best');

        R = [R; table(round(duty*100), T0, Tend, dTm, Km, taum, Lm, R2, rmse, ...
            'VariableNames',{'Duty_pct','T_start_C','T_end_C','DeltaT_C','K_degC_per_duty','Tau_s','Delay_s','R2','RMSE_C'})]; %#ok<AGROW>
    end
    disp('--- Measured / identified parameters ---'); disp(R);
    writetable(R,'surface_measured_results.csv');
    saveas(gcf,'surface_measured_vs_model.png');

    % average identified model (use these in the PID design / Lab5.m)
    K_id = mean(R.K_degC_per_duty); tau_id = mean(R.Tau_s); L_id = mean(R.Delay_s);
    fprintf('\nIdentified model:  G(s) = %.2f * e^(-%.1f s) / (%.1f s + 1)\n', K_id, L_id, tau_id);
    fprintf('Put K = %.2f (positive, cooling delta), tau = %.1f, L = %.1f into Lab5.m\n', abs(K_id), tau_id, L_id);

    % static characteristic with measured points
    figure('Name','Static characteristic (measured)'); hold on; grid on;
    plot(duties*100,Tfinal,'-','LineWidth',1.5);
    plot(R.Duty_pct,R.T_end_C,'ro','LineWidth',1.5,'MarkerSize',8);
    legend('Model','Measured','Location','best');
    xlabel('Duty (%)'); ylabel('Steady-state surface temp (\circC)');
    title('Duty vs steady-state temperature');
    saveas(gcf,'surface_static_measured.png');
end
