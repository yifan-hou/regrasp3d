% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
clear;clc;

addpath ../grasp
addpath ../model
addpath ../model/data

% Constraints
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit


% friction between object and  ground
para.MU = 0.5;

% Grasp sampling
para.NGS            = 200; % number of grasp pos samplings
para.ANGLE_TOL      = 0.1; % rad  % grasp axis tolerance
para.COM_DIST_LIMIT = 0.8; % meter

% planning parameter
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree
para.COLLISION_FREE_ANGLE_MARGIN = 5; % stay away from collsion for at least 5 degrees
									  % has to be an positive integer

% Popups 
para.showObject             = false; % show object and the simplified object
para.showObject_id          = [1 2 3];
para.showStablePoses        = false;
para.showStablePoses_id     = 1;
para.showCheckedGrasp       = false;
para.showCheckedGrasp_id    = 1;
para.showProblem            = false;
para.showProblem_id         = [1 2 3];


% % get object mesh
% file_dir = dir('../model/data/*.stl');
% for i = 6:length(file_dir)
% 	filename = file_dir(i).name;	
% 	disp(['Processing # ' num2str(i) ' of ' num2str(length(file_dir)) ', name: ' filename]);
% 	[fgraph, pgraph, mesh, mesh_s] = getObject(para, filename);
% 	gripper                        = getGripper();
% 	[grasps, fgraph]               = calGrasp(fgraph, pgraph, mesh, mesh_s, gripper, para);
% 	full_path = ['../model/data/' filename(1:end-4) '.mat'];
% 	save(full_path, 'fgraph', 'pgraph', 'mesh', 'mesh_s', 'grasps');
% end

% modify error bound
file_dir = dir('../model/data/*.stl');
for i = 1:length(file_dir)
	filename  = file_dir(i).name;	
	full_path = ['../model/data/' filename(1:end-4) '.mat'];
	load(full_path);
	disp(['Processing # ' num2str(i) ' of ' num2str(length(file_dir)) ', name: ' filename]);
	disp(['err_bound was ' num2str(pgraph.err_bound)]);
	[~, pgraph_amended, ~, ~] = getObject(para, filename);
	pgraph.err_bound          = pgraph_amended.err_bound;
	save(full_path, 'fgraph', 'pgraph', 'mesh', 'mesh_s', 'grasps');
end
