clear;clc;

addpath ../data
addpath ../grasp
addpath ../mesh

[mode_graph, mesh] = getObject();
buildMapEdges(mode_graph, mesh);
return;

% get start, goal orientation
q0 = getOrientation();
qg = getOrientation();



% get modes
m0 = Ori2mod(q0);
mg = Ori2mod(qg);

% 
