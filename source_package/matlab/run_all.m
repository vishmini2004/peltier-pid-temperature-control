function run_all()
%RUN_ALL  Run the whole simulation package in report order.
% Figures go to ../figures (PNG for the report, FIG for editing).
% Requires: MATLAB + Control System Toolbox. Simulink is optional.
% Each script runs in its own workspace (the scripts start with 'clear').
here = fileparts(mfilename('fullpath'));
scripts = {'s01_model_analysis', 's02_before_after', 's03_pid_design', 's04_gain_sweep', ...
           's05_setpoint_tracking', 's06_disturbance_rejection', 's07_robustness', ...
           's08_identify_plant', 's09_sim_vs_experiment'};
for i = 1:numel(scripts)
    fprintf('\n\n######## %s ########\n', scripts{i});
    cd(here);
    run_isolated(scripts{i});
end
cd(here);
if license('test', 'Simulink') && ~isempty(ver('simulink'))
    fprintf('\n\n######## build_simulink_model ########\n');
    build_simulink_model('PID_recommended');
else
    fprintf('\nSimulink not available - skipped build_simulink_model.\n');
end
end

function run_isolated(name)
run(name);
end
