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


% stable pose computation
para.bottom_height_tol = 1; % mm points with z < this number will be considered as contact point
para.minimal_support_polygon_area = 40; % mm^2

% convex hull simplification parameters
para.NF_CVR      = 500; % 80 target number of faces on the simplified convex hull
para.err_tol_CVR = 5;

COM = [0 -9.65 0;
       -3 0.3 36]';


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