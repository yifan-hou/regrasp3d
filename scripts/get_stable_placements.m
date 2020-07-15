% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
close all;clear;clc;

addpath ../planning
addpath ../model
% addpath ../model/test_objects
addpath ../offline
addpath ../ploting
addpath ../model/Kitting

% Constraints
% para.GRIPPER_TILT_LIMIT = 50*pi/180; % tilting angle tolerance
% para.GRIPPER_Z_LIMIT    = 1; % finger position limit

% stable pose computation
para.bottom_height_tol = 1; % mm points with z < this number will be considered as contact point
para.minimal_support_polygon_area = 40; % mm^2


% % Grasp sampling
% para.NGS                   = 100; % maximum number of grasp pos samplings
% para.N_RESAMPLE            = 5;   % if a sequence of N samples are all close to previous grasps, then stop 
% para.ANGLE_TOL             = 3*pi/180; % rad  % grasp axis tolerance
% para.COM_DIST_LIMIT        = 2; % minimum distance between grasp axis and COM
% para.GRASP_DIST_LIMIT      = 5; % minimum distance to other grasps
% para.POINTJ_SAMPLE_DENSITY = 0.03; % # of points per mm^2
% para.grasp_travel          = 90-10;

% convex hull simplification parameters
para.NF_CVR      = 500; % 80 target number of faces on the simplified convex hull
para.err_tol_CVR = 5;

% % collision checking
% para.COLLISION_FREE_ANGLE_MARGIN = 5; % stay away from collsion for at least 5 degrees
% 									  % has to be an positive integer

% Popups 
para.showObject             = true; % show object and the simplified object
para.showObject_id          = [1 2 3];
para.showStablePoses        = true;
para.showStablePoses_id     = 4;
para.showGraspChecking      = true;
para.showGraspChecking_id   = 2;

gripper = getGripper();

% get object mesh
file_dir = dir('../model/Kitting/*.stl');

% COM = [
% 		0 -9.65 0; % big screw
% 	   0 -4.647 13.27; % steel hook
% 	   -10 10 0 % pipe fitting
% 	   ]';
COM = [0 -9.65 0; % big screw
       -3 0.3 36; % bar clamp
       -4.5 0.45 54; % bar clamp 105
	   0.2 -3 37.5; % e stop holder
	   0 2.8 39.6; % nozzle
       0 12.27 0; % nut
	   -0.3 4.6 39; % part1
       0 14.18 0 % timing pulley
	   ]';


for i = 1:length(file_dir)
	% 
	% get file name
	% 
	filename = file_dir(i).name;	
    temp_file_path = [filename(1:end-4) '_temp.mat'];
    data_file_path = ['../model/test_objects/' filename(1:end-4) '.mat'];
	disp(['Processing # ' num2str(i) ' of ' num2str(length(file_dir)) ', name: ' filename]);
    
    % 
    % load the stl file
    % 
	% Read object stl file
	[fgraph, pgraph, mesh, mesh_s] = getObject(para, COM(:, i), filename);
    disp(fgraph.quat);
end