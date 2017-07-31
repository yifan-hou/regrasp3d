function [gripper_plan_2d, qp] = planOneGrasp(mesh, gp1o_w, gp2o_w, q0, qf, pgraph, para )

% -----------------------------------------
% 	Prepare the 2D problem
% -----------------------------------------

% under world coordinate, calculate the grasp pos for object in q0, qf
gp10_w    = quatOnVec(gp1o_w, q0);
gp20_w    = quatOnVec(gp2o_w, q0);
qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate

gp1f_w    = quatOnVec(gp1o_w, qf);
gp2f_w    = quatOnVec(gp2o_w, qf);
qgraspf_w = getProperGrasp(gp1f_w, gp2f_w); % grasp frame for qf, under world coordinate

% roll - pivot - roll
% sample a few orientations for pivoting.
qp = quatSlerp(q0, qf, 0.0); % todo: do clever sampling

% rotate the gripper frame to qp
q0p_w      = quatMTimes(qp, quatInv(q0)); % a rotation that rotates q0 to qp, measured in world frame
qgrasp0p_w = quatMTimes(q0p_w, qgrasp0_w); % grasp 0, rotated to p, measured in world frame
qfp_w      = quatMTimes(qp, quatInv(qf)); % a rotation that rotates qf to qp, measured in world frame 
qgraspfp_w = quatMTimes(qfp_w, qgraspf_w); % grasp g, rotated to p, measured in world frame

if para.showProblem
	plotObject(mesh, para.showProblem_id(1), q0);
	plotGripper(1, q0, [gp1o_w gp2o_w], qgrasp0_w);
	plotObject(mesh, para.showProblem_id(2), qf);
	plotGripper(2, qf, [gp1o_w gp2o_w], qgraspf_w);
	plotObject(mesh, para.showProblem_id(3), qp);
	plotGripper(3, qp, [gp1o_w gp2o_w], qgrasp0p_w);
	plotGripper(3, qp, [gp1o_w gp2o_w], qgraspfp_w);
end

% look at gravity and graspfp_w in grasp0p, do the planning
Rw_grasp0p 	 = quat2m(qgrasp0p_w)'; % world, measured in grasp0p frame
% look at gravity in grasp0p
gravity_grasp0p        = Rw_grasp0p*[0;0;-1]; 
qgraspfp_grasp0p       = quatMTimes(quatInv(qgrasp0p_w), qgraspfp_w); 
graspfpz_grasp0p       = quatOnVec([0 0 1]', qgraspfp_grasp0p);
graspfpz_grasp0p_left  = aaOnVec(graspfpz_grasp0p, para.GRIPPER_TILT_LIMIT, [1 0 0]');
graspfpz_grasp0p_right = aaOnVec(graspfpz_grasp0p, -para.GRIPPER_TILT_LIMIT, [1 0 0]');

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
graspfpz_2d       = Rgrasp0p_2d*graspfpz_grasp0p; 
% 	graspfpz_2d_left  = Rgrasp0p_2d*graspfpz_grasp0p_left; 
% 	graspfpz_2d_right = Rgrasp0p_2d*graspfpz_grasp0p_right; 
COMp_2d           = Rgrasp0p_2d*COMp_grasp0p; 
pointsp_2d        = Rgrasp0p_2d*pointsp_grasp0p; 
gpp_2d            = Rgrasp0p_2d*gpp_grasp0p;

Rw_2d             = Rgrasp0p_2d*Rw_grasp0p;

% -----------------------------------------
% 	Solve the 2D problem
% -----------------------------------------

[object_plan_2d, init_grasp_ang] = plan2DObject(gravity_2d, graspfpz_2d, COMp_2d, pointsp_2d, ... 
                              			     gpp_2d, Rw_2d, pgraph.connectmatrix, para);

if isempty(object_plan_2d)
	gripper_plan_2d = [];
	qp = [];
	return;
end

% Check 1: 2d frame
n = [0 0 1]';
n = -n*object_plan_2d.dir;
goal_grasp_2d = graspfpz_2d;
for ii = 1:length(object_plan_2d.motion)
    goal_grasp_2d = quatOnVec(goal_grasp_2d, aa2quat(object_plan_2d.motion(ii), n));
end
assert(abs(angBTVec(goal_grasp_2d(1:2), gravity_2d(1:2)) - pi) < 1e-7);

% Check 2: grasp frame
n = gp1p_grasp0p - gp2p_grasp0p;
n = -n*object_plan_2d.dir;
goal_grasp_grasp0p = graspfpz_grasp0p;
for ii = 1:length(object_plan_2d.motion)
    goal_grasp_grasp0p = quatOnVec(goal_grasp_grasp0p, aa2quat(object_plan_2d.motion(ii), n));
end
assert(abs(angBTVec(goal_grasp_grasp0p(2:3), gravity_grasp0p(2:3)) - pi) < 1e-7);

% -----------------------------------------
% 	Solve for gripper motion (2D)
% -----------------------------------------
gripper_plan_2d = plan2DGripper(object_plan_2d, init_grasp_ang, para);

m0p_w      = quat2m(q0p_w);
gp10p_w    = m0p_w*gp10_w;
gp20p_w    = m0p_w*gp20_w;
COM0p_w    = m0p_w*quatOnVec(mesh.COM, q0);
points0p_w = m0p_w*quatOnVec(mesh.points, q0);

% Check 3: world frame check
qgrasp_now = qgrasp0p_w;
q_now = qp;
n = gp10p_w - gp20p_w; n = n/norm(n);
n = -n*gripper_plan_2d.dir;
for ii = 1:length(gripper_plan_2d.gripper_motion)
    qgrasp_now = quatMTimes(aa2quat(gripper_plan_2d.gripper_motion(ii), n), qgrasp_now);
    q_now = quatMTimes(aa2quat(gripper_plan_2d.object_motion(ii), n), q_now);
end
q_goal = quatMTimes(quatInv(qp), qgraspfp_w);
q_achieve = quatMTimes(quatInv(q_now), qgrasp_now);

assert(angBTquat(q_goal, q_achieve) < 1e-4); 


end