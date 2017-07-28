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
para.showProblem            = true;
para.showProblem_id         = [1 2 3];
para.show2Dproblem          = false;
para.show2Dproblem_id       = 4;

[fgraph, pgraph, mesh] = getObject(para);
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
	gp2o_w = grasps.points(:,id_common(i), 2);

	% under world coordinate, calculate the grasp pos for object in q0, qg
	gp10_w    = quatOnVec(gp1o_w, q0);
	gp20_w    = quatOnVec(gp2o_w, q0);
	qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate

	gp1g_w    = quatOnVec(gp1o_w, qg);
	gp2g_w    = quatOnVec(gp2o_w, qg);
	qgraspg_w = getProperGrasp(gp1g_w, gp2g_w); % grasp frame for qg, under world coordinate

	% roll - pivot - roll
	% sample a few orientations for pivoting.
	qp = quatSlerp(q0, qg, 0.0); % todo: do clever sampling

	% rotate the gripper frame to qp
	q0p_w      = quatMTimes(qp, quatInv(q0)); % a rotation that rotates q0 to qp, measured in world frame
	qgrasp0p_w = quatMTimes(q0p_w, qgrasp0_w); % grasp 0, rotated to p, measured in world frame
	qgp_w      = quatMTimes(qp, quatInv(qg)); % a rotation that rotates qg to qp, measured in world frame 
	qgraspgp_w = quatMTimes(qgp_w, qgraspg_w); % grasp g, rotated to p, measured in world frame

	if para.showProblem
		plotObject(mesh, para.showProblem_id(1), q0);
		plotGripper(1, q0, [gp1o_w gp2o_w], qgrasp0_w);
		plotObject(mesh, para.showProblem_id(2), qg);
		plotGripper(2, qg, [gp1o_w gp2o_w], qgraspg_w);
		plotObject(mesh, para.showProblem_id(3), qp);
		plotGripper(3, qp, [gp1o_w gp2o_w], qgrasp0p_w);
		plotGripper(3, qp, [gp1o_w gp2o_w], qgraspgp_w);
	end

	% look at gravity and graspgp_w in grasp0p, do the planning
	Rw_grasp0p 	 = quat2m(qgrasp0p_w)'; % world, measured in grasp0p frame
	% look at gravity in grasp0p
	gravity_grasp0p        = Rw_grasp0p*[0;0;-1]; 
	qgraspgp_grasp0p       = quatMTimes(quatInv(qgrasp0p_w), qgraspgp_w); 
	graspgpz_grasp0p       = quatOnVec([0 0 1]', qgraspgp_grasp0p);
	graspgpz_grasp0p_left  = aaOnVec(graspgpz_grasp0p, para.GRIPPER_TILT_LIMIT, [1 0 0]');
	graspgpz_grasp0p_right = aaOnVec(graspgpz_grasp0p, -para.GRIPPER_TILT_LIMIT, [1 0 0]');

	% position of points
	qp_grasp0p      = quatMTimes(quatInv(qgrasp0p_w), qp);
	COMp_grasp0p    = quatOnVec(mesh.COM, qp_grasp0p);
	pointsp_grasp0p = quatOnVec(mesh.points, qp_grasp0p);
	gp1p_grasp0p    = quatOnVec(gp1o_w, qp_grasp0p);
	gp2p_grasp0p    = quatOnVec(gp2o_w, qp_grasp0p);

	COMp_grasp0p    = bsxfun(@minus, COMp_grasp0p, 	gp1p_grasp0p);
	pointsp_grasp0p = bsxfun(@minus, pointsp_grasp0p, gp1p_grasp0p);
	gpp_grasp0p     = [gp1p_grasp0p - gp1p_grasp0p, gp2p_grasp0p - gp1p_grasp0p];

	% change of frame: XYZ -> ZXY (frame grasp0p -> frame 2d)
	Rgrasp0p_2d 	  = [0 1 0;
					     0 0 1;
					     1 0 0];
	gravity_2d        = Rgrasp0p_2d*gravity_grasp0p; 
	graspgpz_2d       = Rgrasp0p_2d*graspgpz_grasp0p; 
% 	graspgpz_2d_left  = Rgrasp0p_2d*graspgpz_grasp0p_left; 
% 	graspgpz_2d_right = Rgrasp0p_2d*graspgpz_grasp0p_right; 
	COMp_2d           = Rgrasp0p_2d*COMp_grasp0p; 
	pointsp_2d        = Rgrasp0p_2d*pointsp_grasp0p; 
	gpp_2d            = Rgrasp0p_2d*gpp_grasp0p;
	
	Rw_2d             = Rgrasp0p_2d*Rw_grasp0p;

	% % test
	% gp1p_w = Rw_grasp0p'*gp1p_grasp0p;
	% gp2p_w = Rw_grasp0p'*gp2p_grasp0p;
	% Rw_2d*(gp1p_w - gp2p_w) % should be [0 0 ?]

	[object_plan, init_grasp_ang] = plan2DObject(gravity_2d, graspgpz_2d, COMp_2d, pointsp_2d, ... 
                                  			     gpp_2d, Rw_2d, pgraph.connectmatrix, para);
    
    if isempty(object_plan)
        disp(['Grasp ' num2str(i) ', No solution']);
        continue;
    else
        disp(['Grasp ' num2str(i)]);
        disp(object_plan);
    end

    % Check 1: 2d frame
	n = [0 0 1]';
	n = -n*object_plan.dir;
	goal_grasp_2d = graspgpz_2d;
    for ii = 1:length(object_plan.motion)
        goal_grasp_2d = quatOnVec(goal_grasp_2d, aa2quat(object_plan.motion(ii), n));
    end
    assert(abs(angBTVec(goal_grasp_2d(1:2), gravity_2d(1:2)) - pi) < 1e-7);

    % Check 2: grasp frame
	n = gp1p_grasp0p - gp2p_grasp0p;
	n = -n*object_plan.dir;
	goal_grasp_grasp0p = graspgpz_grasp0p;
    for ii = 1:length(object_plan.motion)
        goal_grasp_grasp0p = quatOnVec(goal_grasp_grasp0p, aa2quat(object_plan.motion(ii), n));
    end
    assert(abs(angBTVec(goal_grasp_grasp0p(2:3), gravity_grasp0p(2:3)) - pi) < 1e-7);


	% plan 2 gripper motion
	plan = plan2DGripper(object_plan, init_grasp_ang, para);

	m0p_w      = quat2m(q0p_w);
	gp10p_w    = m0p_w*gp10_w;
	gp20p_w    = m0p_w*gp20_w;
	COM0p_w    = m0p_w*quatOnVec(mesh.COM, q0);
	points0p_w = m0p_w*quatOnVec(mesh.points, q0);

    % Check 3: world frame check
	qgrasp_now = qgrasp0p_w;
	q_now = qp;
	n = gp10p_w - gp20p_w; n = n/norm(n);
	n = -n*plan.dir;
    for ii = 1:length(plan.gripper_motion)
        qgrasp_now = quatMTimes(aa2quat(plan.gripper_motion(ii), n), qgrasp_now);
        q_now = quatMTimes(aa2quat(plan.object_motion(ii), n), q_now);
    end
	q_goal = quatMTimes(quatInv(qp), qgraspgp_w);
	q_achieve = quatMTimes(quatInv(q_now), qgrasp_now);

    assert(norm(q_goal - q_achieve) < 1e-4); 

	animatePlan(mesh, 5, gp10p_w, gp20p_w, qgrasp0p_w, COM0p_w, points0p_w, plan);
	% gripper motion closed loop control

end