function add_casadi_path(verbose)
%ADD_CASADI_PATH  Add CasADi to MATLAB path (Windows).
%
% add_casadi_path() defaults to verbose = true.
% add_casadi_path(false) suppresses the "Added CasADi path" print (useful in parfor).

if nargin < 1 || isempty(verbose)
    verbose = true;
end

candidate_paths = {
    'C:\Users\sanja\OneDrive - Louisiana State University\Documents\MATLAB\casadi-3.7.2-windows64-matlab2018b'
    'C:\Users\Aanchal\OneDrive - Louisiana State University\Documents\MATLAB\casadi-3.7.2-windows64-matlab2018b'
    "C:\Users\anime\OneDrive\Desktop\Work\ICORE\ACC_27_Optimal Periodic Control\casadi-3.7.2-windows64-matlab2018b"
};

for ip = 1:numel(candidate_paths)
    if exist(candidate_paths{ip}, 'dir')
        addpath(candidate_paths{ip});
        if verbose
            fprintf('Added CasADi path: %s\n', candidate_paths{ip});
        end
        return;
    end
end

warning('CasADi not found. Edit add_casadi_path.m to your install path.');
end

