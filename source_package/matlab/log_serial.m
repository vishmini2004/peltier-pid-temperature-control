function log_serial(port, outfile, duration_s)
%LOG_SERIAL  Record the firmware's CSV stream to a file with a live plot.
%   log_serial("COM5", "../data/closedloop_run.csv", 2700)
%   Lines starting with '#' (firmware messages) are echoed, not saved.
%   Send commands from the Arduino Serial Monitor BEFORE starting the logger,
%   or use writeline(s, "SP 15") on the returned port in your own script.

if nargin < 3, duration_s = inf; end
s = serialport(port, 115200);
configureTerminator(s, "LF");
flush(s);
fid = fopen(outfile, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 't_s,T_surf,T_hot,T_ref,duty,load,mode\n');

fig = figure('Name', ['Live: ' char(outfile)], 'Color', 'w');
ax1 = subplot(2,1,1); hT = animatedline(ax1, 'LineWidth', 1.3); hR = animatedline(ax1, 'LineStyle', '--');
grid(ax1, 'on'); ylabel(ax1, 'T (\circC)');
ax2 = subplot(2,1,2); hU = animatedline(ax2); grid(ax2, 'on'); ylabel(ax2, 'Duty'); xlabel(ax2, 'Time (s)');

t_start = tic;
while toc(t_start) < duration_s && isvalid(fig)
    line = strtrim(readline(s));
    if strlength(line) == 0, continue; end
    if startsWith(line, "#") || startsWith(line, "t_s"), disp(line); continue; end
    fprintf(fid, '%s\n', line);
    v = split(line, ",");
    if numel(v) < 6, continue; end
    tt = str2double(v(1));
    addpoints(hT, tt, str2double(v(2))); addpoints(hR, tt, str2double(v(4)));
    addpoints(hU, tt, str2double(v(5)));
    drawnow limitrate;
end
clear s;
end
