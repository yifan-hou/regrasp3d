% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
clear;clc;

addpath ../planning
addpath ../model
addpath ../model/test_objects
addpath ../offline

% Constraints
para.GRIPPER_TILT_LIMIT = 80*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 10; % finger position limit

% stable pose computation
para.bottom_height_tol = 1; % mm points with z < this number will be considered as contact point
para.minimal_support_polygon_area = 200; % mm^2


% Grasp sampling
para.NGS                   = 100; % maximum number of grasp pos samplings
para.N_RESAMPLE            = 5;   % if a sequence of N samples are all close to previous grasps, then stop 
para.ANGLE_TOL             = 1*pi/180; % rad  % grasp axis tolerance
para.COM_DIST_LIMIT        = 10; % minimum distance between grasp axis and COM
para.GRASP_DIST_LIMIT      = 5; % minimum distance to other grasps
para.POINTJ_SAMPLE_DENSITY = 0.03; % # of points per mm^2
para.grasp_travel          = 90-10;

% convex hull simplification parameters
para.NF_CVR      = 500; % 80 target number of faces on the simplified convex hull
para.err_tol_CVR = 5;

% collision checking
para.COLLISION_FREE_ANGLE_MARGIN = 5; % stay away from collsion for at least 5 degrees
									  % has to be an positive integer

% Popups 
para.showObject             = true; % show object and the simplified object
para.showObject_id          = [1 2 3];
para.showStablePoses        = true;
para.showStablePoses_id     = 4;
para.showGraspChecking      = true;
para.showGraspChecking_id   = 2;

gripper = getGripper();

% get object mesh
file_dir = dir('../model/test_objects/*.stl');

% COM = [
% 		0 -9.65 0; % big screw
% 	   0 -4.647 13.27; % steel hook
% 	   -10 10 0 % pipe fitting
% 	   ]';
COM = [-3 0.3 36; % bar clamp
	   0.2 -3 37.5; % e stop holder
	   0 2.8 39.6; % nozzle
	   -0.3 4.6 39 % part1
	   ]';


for i = 4:length(file_dir)
	% 
	% get file name
	% 
	filename = file_dir(i).name;	
    full_path = ['../model/test_objects/' filename(1:end-4) '.mat'];
	disp(['Processing # ' num2str(i) ' of ' num2str(length(file_dir)) ', name: ' filename]);
    
    % 
    % load the stl file
    % 
	% Read object stl file
% 	[fgraph, pgraph, mesh, mesh_s] = getObject(para, COM(:, i), filename);
%     save debug fgraph pgraph mesh mesh_s
	% % or, load existed object files
	load debug; 

	% 
	% compute grasps
	% 
    % [grasps, fgraph] = calGrasp(fgraph, pgraph, mesh, mesh_s, gripper, para);
	clickGrasp(mesh_s, gripper, para);
    
    % breakpoint here!
    disp('Put break point here!!!');
    
	[grasps, fgraph] = checkGrasp4StableMode(fgraph, pgraph, mesh, para);


	save(full_path, 'fgraph', 'pgraph', 'mesh', 'mesh_s', 'grasps', 'gripper');
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
% % 	save(full_path, 'fgraph', 'pgraph', 'mesh', 'mesh_s', 'grasps');
% end
