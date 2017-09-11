% do planning given:
% 	grasp_id: a specific grasp pos
% 	q0, qf: initial/final object pose
% 	qg0, qgf: initial/final grasp pose (could be empty)
function [plan_2d, flag] = planOneGrasp(mesh, grasps, grasp_id, q0, qf, qg0, qgf, pgraph, para )

% -----------------------------------------
% 	Get the grasps
% -----------------------------------------

% under world coordinate, calculate the grasp pos for object in q0, qf
gp1o_w = grasps.points(:, grasp_id, 1);
gp2o_w = grasps.points(:, grasp_id, 2);

gp10_w = quatOnVec(gp1o_w, q0);
gp20_w = quatOnVec(gp2o_w, q0);
if isempty(qg0)
	[qgrasp0_w, qgrasp0frame_w] = getProperGrasp(gp10_w, gp20_w, grasps.range(:, grasp_id), q0, gp1o_w, gp2o_w, para); % grasp frame for q0, under world coordinate
else
	[~, qgrasp0frame_w] = getProperGrasp(gp10_w, gp20_w, grasps.range(:, grasp_id), q0, gp1o_w, gp2o_w, para); % grasp frame for q0, under world coordinate
	qgrasp0_w           = qg0;
end

gp1f_w = quatOnVec(gp1o_w, qf);
gp2f_w = quatOnVec(gp2o_w, qf);
if isempty(qgf)
	[qgraspf_w, qgraspfframe_w] = getProperGrasp(gp1f_w, gp2f_w, grasps.range(:, grasp_id), qf, gp1o_w, gp2o_w, para); % grasp frame for qf, under world coordinate
else
	[~, qgraspfframe_w] = getProperGrasp(gp1f_w, gp2f_w, grasps.range(:, grasp_id), qf, gp1o_w, gp2o_w, para); % grasp frame for qf, under world coordinate
	qgraspf_w           = qgf;
end

% check grasp frame
temp  = gp10_w - gp20_w;
tempv = quatOnVec([1 0 0]', qgrasp0_w);
assert( abs(angBTVec(temp, tempv)) < 1e-7);
plan_2d = []; qp = [];
if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qgrasp0frame_w)) > para.GRIPPER_TILT_LIMIT)
	flag = -3;
	return;
end
if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qgraspfframe_w)) > para.GRIPPER_TILT_LIMIT)
	flag = -3;
	return;
end
if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qgrasp0_w)) > para.GRIPPER_TILT_LIMIT)
	flag = -3;
	return;
end
if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qgraspf_w)) > para.GRIPPER_TILT_LIMIT)
	flag = -3;
	return;
end


% find out the slice of collision-free range
ax0_w                = quatOnVec([1 0 0]', qgrasp0_w); % grasp axis in world frame
gripper0_w           = quatOnVec([0 0 1]', qgrasp0_w); % initial gripper orientation in world frame
ref_frame            = quatMTimes(q0, grasps.ref_frame(:, grasp_id));
ref                  = quatOnVec([0 0 1]', ref_frame);
init_sf_id           = angBTVec(ref, gripper0_w, ax0_w, 1);
init_sf_id           = floor(180/pi*init_sf_id) + 1;
collision_free_range = grasps.range(:, grasp_id);
collision_free_range = singleOutSFRange(collision_free_range, init_sf_id);
assert(circQuery(collision_free_range, init_sf_id) == 1);

% roll - pivot - roll
% sample a few orientations for pivoting.
qp = quatSlerp(q0, qf, 0.0); % todo: do clever sampling

% ----------------------------------------------
% 	Rotate to qp
% ----------------------------------------------
gp1p_w    = quatOnVec(gp1o_w, qp);
gp2p_w    = quatOnVec(gp2o_w, qp);
[~, qP_w] = getProperGrasp(gp1p_w, gp2p_w); % p frame, measured in world frame

% calculate the gripper motion constraints for 2D problem
zP_w               = quatOnVec([0 0 1]', qP_w); % z axis of p frame, measured in w frame
tilted_ang         = angBTVec([0 0 1]', zP_w);
gripper_cone_width = tiltedConeAng(para.GRIPPER_TILT_LIMIT, tilted_ang);
if tilted_ang > para.GRIPPER_TILT_LIMIT
    flag = -3;
	return;
end

q0p_w      = quatMTimes(qp, quatInv(q0));  % a rotation that rotates q0 to qp, measured in world frame
qgrasp0p_w = quatMTimes(q0p_w, qgrasp0_w); % grasp 0, rotated to p, measured in world frame
qfp_w      = quatMTimes(qp, quatInv(qf));  % a rotation that rotates qf to qp, measured in world frame 
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

% ----------------------------------------------
% look in p frame
% ----------------------------------------------
% orientations
Rw_P       = quat2m(qP_w)'; % world, measured in p frame
gravity_P  = Rw_P*[0; 0; -1]; 
qgrasp0p_P = quatMTimes(quatInv(qP_w), qgrasp0p_w); 
qgraspfp_P = quatMTimes(quatInv(qP_w), qgraspfp_w); 
grasp0pz_P = quatOnVec([0 0 1]', qgrasp0p_P);
graspfpz_P = quatOnVec([0 0 1]', qgraspfp_P);

% positions (rotated by qp, then measured in p frame)
qp_P      = quatMTimes(quatInv(qP_w), qp); % pframe and qp are different!
COMp_P    = quatOnVec(mesh.COM, qp_P);
pointsp_P = quatOnVec(pgraph.vertices, qp_P);
gp1p_P    = quatOnVec(gp1o_w, qp_P); % note: gp1o_w = gp1p_p
gp2p_P    = quatOnVec(gp2o_w, qp_P);


% ----------------------------------------------
% 	Look in 2d frame
% ----------------------------------------------
% translational offset
COMp_P    = bsxfun(@minus, COMp_P, 	gp1p_P);
pointsp_P = bsxfun(@minus, pointsp_P, gp1p_P);
gpp_P     = [gp1p_P - gp1p_P, gp2p_P - gp1p_P];

% change of frame: XYZ -> ZXY (frame grasp0p -> frame 2d)
Rp_2d  			  = [0 1 0;
				     0 0 1;
				     1 0 0];
gravity_2d        = Rp_2d*gravity_P; 
grasp0pz_2d       = Rp_2d*grasp0pz_P; 
graspfpz_2d       = Rp_2d*graspfpz_P; 
COMp_2d           = Rp_2d*COMp_P; 
pointsp_2d        = Rp_2d*pointsp_P; 
gpp_2d            = Rp_2d*gpp_P;
Rw_2d             = Rp_2d*Rw_P;

% -----------------------------------------
% 	Solve the 2D problem 
% 	for object motion
% -----------------------------------------
[object_plan_2d, init_grasp_ang, flag] = plan2DObject(gravity_2d, grasp0pz_2d, graspfpz_2d, COMp_2d, gpp_2d, Rw_2d, pointsp_2d, ...
										 pgraph.err_bound, collision_free_range, init_sf_id, ...
										 gripper_cone_width, para);

if flag <= 0
	plan_2d = [];
	qp      = [];
	return;
end

% Check 1: 2d frame
n             = [0 0 1]';
n             = -n*object_plan_2d.dir;
goal_grasp_2d = graspfpz_2d;
for ii = 1:length(object_plan_2d.motion)
    goal_grasp_2d = quatOnVec(goal_grasp_2d, aa2quat(object_plan_2d.motion(ii), n));
end
assert(abs(pi - angBTVec(goal_grasp_2d(1:2), gravity_2d(1:2)) - gripper_cone_width) < 1e-4);


% Check 2: p frame: move goal to edge of cone
n_P          = gp1p_P - gp2p_P; n_P = n_P/norm(n_P);
objang       = sum(object_plan_2d.motion);
goal_grasp_p = quatOnVec(graspfpz_P, aa2quat(objang, -n_P*object_plan_2d.dir));
assert(abs(pi - angBTVec(goal_grasp_p(2:3), gravity_P(2:3)) - gripper_cone_width) < 1e-4);

% check 3: world frame, rotate object by object_plan_2d.motion, goal is at gripper_cone, check their angle

n_w          = gp1p_w - gp2p_w; n_w = n_w/norm(n_w);
gripper_cone = quatOnVec(zP_w, aa2quat(-object_plan_2d.dir*gripper_cone_width, n_w));

qgoal_rot_w = quatMTimes(aa2quat(objang, -n_w*object_plan_2d.dir), qgraspfp_w);
goal_now    = quatOnVec([0 0 1]', qgoal_rot_w);
assert(angBTVec(gripper_cone, goal_now) < 5e-3);

% q_now      = quatMTimes(aa2quat(objang, -n_w*object_plan_2d.dir), qp);
% object_now = quatOnVec([0 0 1]', q_now);
% assert(abs(angBTVec(gripper_cone, object_now) - angBTquat(qgraspfp_w, qp)) < 1e-3);

% -----------------------------------------
% 	Solve for gripper motion (2D)
% -----------------------------------------
plan_2d    = plan2DGripper(object_plan_2d, init_grasp_ang, gripper_cone_width);

if isempty(plan_2d)
	flag    = -5;
	qp      = [];
	return;
end

% check 4: gripper rotation angle
gripper_motion_togo = angBTquat(qgrasp0p_w, qgraspfp_w);
gripper_motion_cal  = sum(plan_2d.grp_motion_diff) + plan_2d.dir*sum(plan_2d.obj_motion_diff);
assert( abs(gripper_motion_togo - abs(ang_pi(gripper_motion_cal))) < 1e-3);

% Check 3: world frame check
qgrasp_now = qgrasp0p_w;
q_now      = qp; 

for ii = 1:length(plan_2d.grp_motion_diff)
	qgrasp_now = quatMTimes(aa2quat(plan_2d.grp_motion_diff(ii), n_w), qgrasp_now);
	q_now      = quatMTimes(aa2quat(plan_2d.obj_motion_diff(ii), -n_w*plan_2d.dir), q_now);
end
q_goal    = quatMTimes(quatInv(qp), qgraspfp_w);
q_achieve = quatMTimes(quatInv(q_now), qgrasp_now);

assert(angBTquat(q_goal, q_achieve) < 1e-3); 

plan_2d.q0 = q0;
plan_2d.qf = qf;
plan_2d.qp = qp;
plan_2d.grasp_id = grasp_id;

end


% set sf_range to all zero, except the zone of 1 that constains id
function sf_range = singleOutSFRange(sf_range, id)
N = length(sf_range);
ids = false(N,1);

for i = id:-1:1
	if sf_range(i) == 1
		ids(i) = true;
	else
		break;
	end
end

for i = id+1:N
	if sf_range(i) == 1
		ids(i) = true;
	else
		break;
	end
end

sf_range      = false(N, 1);
sf_range(ids) = true;

end



function ang = tiltedConeAng(cone_half_ang, tilted_ang)
H = 1;
R = tan(cone_half_ang)*H;
r = tan(tilted_ang)*H;
L = sqrt(R^2 + H^2);
m = sqrt(r^2 + H^2);
theta = asin(r/R);
h = R*cos(theta);

ang = sss2aaa(h, L, m);

end