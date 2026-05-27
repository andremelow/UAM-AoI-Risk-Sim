function setup_paths()
% SETUP_PATHS  Adds all simulator subdirectories to the MATLAB path.
%
%  Run once per session:  >> setup_paths
%  Or call at the top of run_sim.m.

thisDir = fileparts(mfilename('fullpath'));

addpath(fullfile(thisDir, 'config'));
addpath(fullfile(thisDir, 'core'));
addpath(fullfile(thisDir, 'policies'));
addpath(fullfile(thisDir, 'viz'));
addpath(fullfile(thisDir, 'util'));

fprintf('UAM-AoI-Risk-Sim: paths configured.\n');
end
