function save_fig(fig, name)
%SAVE_FIG  Save a figure as PNG (report) and FIG (editable) in ../figures.
here = fileparts(mfilename('fullpath'));
outdir = fullfile(here, '..', 'figures');
if ~exist(outdir, 'dir'), mkdir(outdir); end
exportgraphics(fig, fullfile(outdir, [name '.png']), 'Resolution', 200);
savefig(fig, fullfile(outdir, [name '.fig']));
end
