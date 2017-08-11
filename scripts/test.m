global para fgraph pgraph mesh mesh_s grasps gripper q0 qf
clc;
% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit
% friction between object and  ground
para.MU = 0.5;
% number of grasp pos samplings
para.NGS = 5; 
% grasp axis tolerance
para.ANGLE_TOL = 0.1; % rad
para.COM_DIST_LIMIT = 0.8; % meter
% shape of gripper(for collision checking)
% para.gripper_shape = getGripper();
para.GOALSAMPLEDENSITY2D = 15*pi/180; % 1 sample every 5 degree
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree

% Popups 
para.showObject             = false; % show object and the simplified object
para.showObject_id          = [1 2 3];
para.showAllGraspSamples    = false;
para.showAllGraspSamples_id = 1;
para.showCheckedGrasp       = false;
para.showCheckedGrasp_id    = 3;
para.showProblem            = false;
para.showProblem_id         = [1 2 3];
para.show2Dproblem          = false;
para.show2Dproblem_id       = 4;

% get object mesh
filename = 'planefrontstay.stl';
% filename = 'sandpart2.stl';
[fgraph, pgraph, mesh, mesh_s] = getObject(para, filename);
gripper = getGripper();


% calculate grasps, and contact mode graph
[grasps, fgraph] = calGrasp(fgraph, pgraph, mesh, mesh_s, gripper, para);

q0 = [1 0 0 0]';
qf = [1 0 0 0]';

disp('[Load Model] Model is loaded.');




return;




clfAll;clear;clc;

addpath ../grasp
addpath ../model
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit
% friction between object and  ground
para.MU = 0.5;
% number of grasp pos samplings
para.NGS = 100; 
% grasp axis tolerance
para.ANGLE_TOL = 0.1; % rad
para.COM_DIST_LIMIT = 0.8; % meter
% shape of gripper(for collision checking)
% para.gripper_shape = getGripper();
para.GOALSAMPLEDENSITY2D         = 15*pi/180; % 1 sample every 5 degree
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree

% Popups 
para.showObject             = false; % show object and the simplified object
para.showObject_id          = [1 2 3];
para.showAllGraspSamples    = false;
para.showAllGraspSamples_id = 1;
para.showCheckedGrasp       = false;
para.showCheckedGrasp_id    = 3;
para.showProblem            = false;
para.showProblem_id         = [1 2 3];
para.show2Dproblem          = false;
para.show2Dproblem_id       = 4;

% get object mesh
filename = 'planefrontstay.stl';


[fgraph, pgraph, mesh] = getObject(para, filename);
gripper                = getGripper();

gp              = [0.5 -0.5; 0 0; 0.5 0.5];

qgrasp = getProperGrasp(gp(:,1), gp(:,2)); 

lookuptable 		    = getIntersectionLookUpTable();
fingertip_plus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices{1}, qgrasp), gp(:, 1))';
fingertip_plus.faces    = gripper.faces{1};
[~, surf1]              = SurfaceIntersectionMex(mesh, fingertip_plus, lookuptable);

disp('Done');

