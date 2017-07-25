clfAll;clear;clc;


addpath ../data
addpath ../grasp
addpath ../mesh

% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit
% friction between object and  ground
para.MU = 0.5;
% grasp pos sample density
para.GS_DENSITY = 0.1; % 1 point every 0.1 m^2
% grasp axis tolerance
para.ANGLE_TOL = 0.1; % rad
para.COM_DIST_LIMIT = 0.8; % meter
% shape of gripper(for collision checking)
% para.gripper_shape = getGripper();
para.GOALSAMPLEDENSITY2D = 15*pi/180; % 1 sample every 5 degree
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree

para.showObject             = false;
para.showObject_id          = 1;
para.showAllGraspSamples    = false;
para.showAllGraspSamples_id = 1;
para.showCheckedGrasp       = false;
para.showCheckedGrasp_id    = 3;
para.show2Dproblem			= false;
para.show2Dproblem_id		= 4;

[fgraph, pgraph, mesh]   = getObject(para);
[grasps, fgraph] = calGrasp(fgraph, mesh, para);


% -----------------------------------------------
% 		Online computation
% -----------------------------------------------

% get start, goal orientation
q0 = [1 0 0 0]';
% qg = getOrientation();
% save qg.mat qg
load qg

% algorithm begin
grasp_id_0 = checkGrasp(grasps, mesh, q0, para);
grasp_id_g = checkGrasp(grasps, mesh, qg, para);



disp('Trying 0 place and pick');
id_common = find(grasp_id_g == grasp_id_0);
if isempty(id_common)
	disp('No solution for 0 place and pick.');
	return;
end
disp('Number of common grasps:');
disp(length(id_common));

for i = 1:length(id_common)
	gp1o_w = grasps.points(:,id_common(i), 1);
	gp20_w = grasps.points(:,id_common(i), 2);

	% under world coordinate, calculate the grasp pos for object in q0, qg
	gp10_w    = quatOnVec(gp1o_w, q0);
	gp20_w    = quatOnVec(gp20_w, q0);
	qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate

	gp1g_w    = quatOnVec(gp1o_w, qg);
	gp2g_w    = quatOnVec(gp20_w, qg);
	qgraspg_w = getProperGrasp(gp1g_w, gp2g_w); % grasp frame for qg, under world coordinate

	% roll - pivot - roll
	% sample a few orientations for pivoting.
	qp = quatSlerp(q0, qg, 0.0); % todo: do clever sampling

	% rotate the gripper frame to qp
	q0p_w      = quatMTimes(qp, quatInv(q0)); % a rotation that rotates q0 to qp, measured in world frame
	qgrasp0q_w = quatMTimes(q0p_w, qgrasp0_w); % grasp 0, rotated to q, measured in world frame
	qgp_w      = quatMTimes(qp, quatInv(qg)); % a rotation that rotates qg to qp, measured in world frame 
	qgraspgq_w = quatMTimes(qgp_w, qgraspg_w); % grasp g, rotated to q, measured in world frame

% 	plotObject(mesh, 1, q0);
% 	plotGripper(1, q0, [gp1o_w gp20_w], qgrasp0_w);
% 	plotObject(mesh, 2, qg);
% 	plotGripper(2, qg, [gp1o_w gp20_w], qgraspg_w);
% 	plotObject(mesh, 3, qp);
% 	plotGripper(3, qp, [gp1o_w gp20_w], qgrasp0q_w);
% 	plotGripper(3, qp, [gp1o_w gp20_w], qgraspgq_w);

	% look at gravity and graspgq_w in grasp0q, do the planning
	Rw_grasp0q 	 = quat2m(qgrasp0q_w)'; % world, measured in grasp0q frame
	% look at gravity in grasp0q
	gravity_grasp0q        = Rw_grasp0q*[0;0;-1]; 
	qgraspgq_grasp0q       = quatMTimes(quatInv(qgrasp0q_w), qgraspgq_w); 
	graspgqz_grasp0q       = quatOnVec([0 0 1]', qgraspgq_grasp0q);
	graspgqz_grasp0q_left  = aaOnVec(graspgqz_grasp0q, para.GRIPPER_TILT_LIMIT, [1 0 0]');
	graspgqz_grasp0q_right = aaOnVec(graspgqz_grasp0q, -para.GRIPPER_TILT_LIMIT, [1 0 0]');

	% position of points
	qp_grasp0q      = quatMTimes(quatInv(qgrasp0q_w), qp);
	COMp_grasp0q    = quatOnVec(mesh.COM, qp_grasp0q);
	pointsp_grasp0q = quatOnVec(mesh.points, qp_grasp0q);
	gp1p_grasp0q    = quatOnVec(gp1o_w, qp_grasp0q);
	gp2p_grasp0q    = quatOnVec(gp20_w, qp_grasp0q);

	COMp_grasp0q    = bsxfun(@minus, COMp_grasp0q, 	gp1p_grasp0q);
	pointsp_grasp0q = bsxfun(@minus, pointsp_grasp0q, gp1p_grasp0q);
	gpp_grasp0q     = [gp1p_grasp0q - gp1p_grasp0q, gp2p_grasp0q - gp1p_grasp0q];

	% change of frame: XYZ -> ZXY (frame grasp0q -> frame 2d)
	Rgrasp0q_2d 	  = [0 1 0;
					     0 0 1;
					     1 0 0];
	gravity_2d        = Rgrasp0q_2d*gravity_grasp0q; 
	graspgqz_2d       = Rgrasp0q_2d*graspgqz_grasp0q; 
	graspgqz_2d_left  = Rgrasp0q_2d*graspgqz_grasp0q_left; 
	graspgqz_2d_right = Rgrasp0q_2d*graspgqz_grasp0q_right; 
	COMp_2d           = Rgrasp0q_2d*COMp_grasp0q; 
	pointsp_2d        = Rgrasp0q_2d*pointsp_grasp0q; 
	gpp_2d            = Rgrasp0q_2d*gpp_grasp0q;
	
	Rw_2d             = Rgrasp0q_2d*Rw_grasp0q;

	% % test
	% gp1p_w = Rw_grasp0q'*gp1p_grasp0q;
	% gp2p_w = Rw_grasp0q'*gp2p_grasp0q;
	% Rw_2d*(gp1p_w - gp2p_w) % should be [0 0 ?]

	pivotMotion       = plan2DObject(gravity_2d, graspgqz_2d, graspgqz_2d_left, graspgqz_2d_right, ... 
								      COMp_2d, pointsp_2d, gpp_2d, Rw_2d, pgraph.connectmatrix, para);
    
    if isempty(pivotMotion)
        disp(['Grasp ' num2str(i) ', No solution']);
    else
        disp(['Grasp ' num2str(i)]);
        disp(pivotMotion);
    end
	% plan 2 gripper motion

	% gripper motion closed loop control

end