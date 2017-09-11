% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
clear;clc;

addpath ../planning
addpath ../model
addpath ../model/data

% Constraints
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit


% friction between object and  ground
para.MU = 0.5;

% Grasp sampling
para.NGS                   = 100; % maximum number of grasp pos samplings
para.N_RESAMPLE            = 5;   % if a sequence of N samples are all close to previous grasps, then stop 
para.ANGLE_TOL             = 30*pi/180; % rad  % grasp axis tolerance
para.COM_DIST_LIMIT        = 0.15; % minimum distance between grasp axis and COM
para.GRASP_DIST_LIMIT      = 0.1; % minimum distance to other grasps
para.POINTJ_SAMPLE_DENSITY = 100; % # of points per m^2

% convex hull simplification parameters
para.NF_CVR = 80; % target number of faces on the simplified convex hull
para.err_tol_CVR = 0.02;

% planning parameter
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree
para.COLLISION_FREE_ANGLE_MARGIN = 5; % stay away from collsion for at least 5 degrees
									  % has to be an positive integer

% Popups 
para.showObject             = true; % show object and the simplified object
para.showObject_id          = [1 2 3];
para.showStablePoses        = false;
para.showStablePoses_id     = 1;
para.showCheckedGrasp       = false;
para.showCheckedGrasp_id    = 1;
para.showGraspChecking      = false;
para.showGraspChecking_id   = 2;
para.showProblem            = false;
para.showProblem_id         = [1 2 3];


% get object mesh
file_dir = dir('../model/data/*.stl');
for i = 1:length(file_dir)
	filename = file_dir(i).name;	
    full_path = ['../model/data/' filename(1:end-4) '.mat'];
    
	disp(['Processing # ' num2str(i) ' of ' num2str(length(file_dir)) ', name: ' filename]);
    
%    use the following two lines for computing object and gripper
	% [fgraph, pgraph, mesh, mesh_s] = getObject(para, filename);
	% gripper                        = getGripper();
%	use the following line to load object/gripper file
	load(full_path); % only use object file

    [grasps, fgraph]               = calGrasp(fgraph, pgraph, mesh, mesh_s, gripper, para);
	
	save(full_path, 'fgraph', 'pgraph', 'mesh', 'mesh_s', 'grasps', 'gripper');
% 	return;
end

% % modify error bound
% file_dir = dir('../model/data/*.stl');
% for i = 1:length(file_dir)
% 	filename  = file_dir(i).name;	
% 	full_path = ['../model/data/' filename(1:end-4) '.mat'];
% 	load(full_path);
% 	disp(['Processing # ' num2str(i) ' of ' num2str(length(file_dir)) ', name: ' filename]);
% 	disp(['err_bound was ' num2str(pgraph.err_bound)]);
% 	[~, pgraph_amended, ~, ~] = getObject(para, filename);
%     drawnow
% 	pgraph.err_bound          = pgraph_amended.err_bound;
% 	save(full_path, 'fgraph', 'pgraph', 'mesh', 'mesh_s', 'grasps');
% end
