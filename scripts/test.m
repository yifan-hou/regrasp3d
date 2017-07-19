clear;clc;

addpath ../data
addpath ../grasp
addpath ../mesh

para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.4; % finger position limit
% friction between object and  ground
para.MU = 0.5;
% grasp pos sample density
para.GS_DENSITY = 0.1; % 1 point every 0.1 m^2
% grasp axis tolerance
para.ANGLE_TOL = 0.1; % rad
% shape of gripper(for collision checking)
% para.gripper_shape = getGripper();

para.verbosity = 2; % 1: text and figure 0: only text

[graph, mesh]   = getObject(0);
[grasps, graph] = calGrasp(graph, mesh, para);


% get start, goal orientation
q0 = getOrientation();
qg = getOrientation();



% get modes
m0 = Ori2mod(q0);
mg = Ori2mod(qg);

% 
