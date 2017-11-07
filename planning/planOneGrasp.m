% do planning given:
% 	q0, qf: initial/final object pose
% 	grasp_id: a specific grasp pos
% 	qg0, qgf: initial/final grasp pose (could be empty)
function [plan_2d, flag] = planOneGrasp(mesh, grasps, grasp_id, q0, qf, qg0, qgf, pgraph, method, para )
plan_2d = []; 

% -----------------------------------------
% 	Get the grasps
% -----------------------------------------

% under world coordinate, calculate the grasp pos for object in q0, qf
gp1o_w = grasps.points(:, grasp_id, 1);
gp2o_w = grasps.points(:, grasp_id, 2);

gp10_w = quatOnVec(gp1o_w, q0);
gp20_w = quatOnVec(gp2o_w, q0);
gp1f_w = quatOnVec(gp1o_w, qf);
gp2f_w = quatOnVec(gp2o_w, qf);

if strcmp(method, 'pickplace')
	gripper_cone_width = getTiltedGripperCone(gp10_w, gp20_w, para.GRIPPER_TILT_LIMIT);
	if isempty(gripper_cone_width)
	    flag = -3;
		return;
	end

	% get range
	[~, ~, id_center, range0, qg0frame] = getProperGrasp(gp10_w, gp20_w, grasps.range(:, grasp_id), q0, gp1o_w, gp2o_w, para, gripper_cone_width); 
	if ~isempty(qg0)
		v  	   = gp10_w - gp20_w; v  = v/norm(v);
		z0     = quatOnVec([0 0 1]', qg0frame);
		z1     = quatOnVec([0 0 1]', qg0);
		angqg0 = round(angBTVec(z0, z1, v, 1)*180/pi); % 0~360

		angqg0_zone = circQuery(range0, angqg0);
		range0      = circQuery(0*range0, angqg0, angqg0_zone);
	end

	[~, ~, ~,         rangef, qgfframe] = getProperGrasp(gp1f_w, gp2f_w, grasps.range(:, grasp_id), qf, gp1o_w, gp2o_w, para, gripper_cone_width); 
	if ~isempty(qgf)
		v 	   = gp1f_w - gp2f_w; v  = v/norm(v);
		z0     = quatOnVec([0 0 1]', qgfframe);
		z1     = quatOnVec([0 0 1]', qgf);
		angqgf = round(angBTVec(z0, z1, v, 1)*180/pi); % 0~360

		angqgf_zone = circQuery(rangef, angqgf);
		rangef      = circQuery(0*range0, angqgf, angqgf_zone);
	end

	range_ = range0 & rangef;
	if ~any(range_)
		% no grasp available
		flag = -3;
		return;
	end

	id1       = find(range_);
	[~, id2]  = min(abs(id1 - id_center));
	ang       = id1(id2)*pi/180;
	qg_ref    = getProperGrasp(gp1o_w, gp2o_w);
	qgrasp0_w = quatMTimes(aa2quat(ang, gp1o_w-gp2o_w), qg_ref);
	qgrasp0_w = quatMTimes(q0, qgrasp0_w);

	plan_2d.q0               = q0;
	plan_2d.qp               = q0;
	plan_2d.qf               = qf;
	plan_2d.grasp_id         = grasp_id;
	plan_2d.qgrp0            = qgrasp0_w;
	plan_2d.obj_motion_diff  = 0;
	plan_2d.grp_motion_diff  = 0;
	plan_2d.rtype            = 0;
	plan_2d.dir              = 1;
	plan_2d.obj_sliding_acc  = 0;
	plan_2d.obj_sliding_type = 1;

	flag = 1;
	return;
end


if isempty(qg0)
	% grasp frame for q0, under world coordinate
	[qgrasp0_w, qgrasp0frame_w] = getProperGrasp(gp10_w, gp20_w, grasps.range(:, grasp_id), q0, gp1o_w, gp2o_w, para); 
else
	% grasp frame for q0, under world coordinate
	[~, qgrasp0frame_w] = getProperGrasp(gp10_w, gp20_w, grasps.range(:, grasp_id), q0, gp1o_w, gp2o_w, para); 
	qgrasp0_w           = qg0;
end

if isempty(qgf)
	% grasp frame for qf, under world coordinate
	[qgraspf_w, qgraspfframe_w] = getProperGrasp(gp1f_w, gp2f_w, grasps.range(:, grasp_id), qf, gp1o_w, gp2o_w, para); 
else
	% grasp frame for qf, under world coordinate
	[~, qgraspfframe_w] = getProperGrasp(gp1f_w, gp2f_w, grasps.range(:, grasp_id), qf, gp1o_w, gp2o_w, para); 
	qgraspf_w           = qgf;
end

% check grasp frame
temp  = gp10_w - gp20_w;
tempv = quatOnVec([1 0 0]', qgrasp0_w);
assert( abs(angBTVec(temp, tempv)) < 1e-7);

if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qgrasp0_w)) > para.GRIPPER_TILT_LIMIT)
	flag = -3;
	return;
end
if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qgraspf_w)) > para.GRIPPER_TILT_LIMIT)
	flag = -3;
	return;
end

% -----------------------------------------
% 	Find out the slice of collision-free range
% -----------------------------------------
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
% 	Rotate grasps to qp
% ----------------------------------------------
gp1p_w    = quatOnVec(gp1o_w, qp);
gp2p_w    = quatOnVec(gp2o_w, qp);
[~, qP_w] = getProperGrasp(gp1p_w, gp2p_w); % p frame, measured in world frame

% calculate the gripper motion constraints for 2D problem
zP_w               = quatOnVec([0 0 1]', qP_w); % z axis of p frame, measured in w frame
gripper_cone_width = getTiltedGripperCone(gp1p_w, gp2p_w, para.GRIPPER_TILT_LIMIT);
if isempty(gripper_cone_width)
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
% 	Look in p frame
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
% 	Look in Proper frame
% 	(Rotate gravity to points down)
% -----------------------------------------
R2d_pro     = matBTVec([gravity_2d(1:2); 0], [0 -1 0]');
gravity_pro = R2d_pro*gravity_2d; % should be [0 -1 ?]
pointsp_pro = R2d_pro*pointsp_2d;
gpp_pro     = R2d_pro*gpp_2d;
COMp_pro    = R2d_pro*COMp_2d;
goal_pro    = R2d_pro*graspfpz_2d;  % goal(3) should = 0
init_pro    = R2d_pro*grasp0pz_2d;  % init(3) should = 0
assert(abs(init_pro(3)) < 1e-3);
assert(abs(goal_pro(3)) < 1e-3);

% check initial grasp orientation
if norm(gravity_pro(1:2)) < 1e-2
	warning('Grasp axis is too flat');
    flag             = -3;
	return;
end
if init_pro(2) < cos(gripper_cone_width)
	% warning('initial grasp pose is out of constraint')
    flag             = -3;
	return;
end

init_grasp_ang = angBTVec([0 1 0]', init_pro, [0 0 1]');
cf_zero_id     = init_sf_id - round(180/pi*init_grasp_ang);


if para.show2Dproblem
    [~, cp_id] = max(gravity_pro'*pointsp_pro); % id of contact point in points list
    cp         = pointsp_pro(:,cp_id);
    
    figure(para.show2Dproblem_id); clf; hold on;
    h_gravity   = plot(com(1)+[0 g(1)], com(2)+[0 g(2)], '-r', 'linewidth', 2);
    h_com       = plot(com(1), com(2), '.r', 'markersize', 30);
    h_cp        = plot(cp(1), cp(2), '.k', 'markersize', 20);
    h_ps        = plot(ps(1,:), ps(2,:), '.b', 'markersize', 10);
    h_goal      = plot([0 goal(1)], [0 goal(2)], '-g*', 'markersize',1, 'linewidth', 2);
    h_fingertip = plot(0, 0, '.g', 'markersize',30);
    
    plot([0 init(1)], [0 init(2)], '-b*', 'markersize',1, 'linewidth', 2);
    plot([0 gripper_cone_right(1)], [0 gripper_cone_right(2)], '-.b', 'markersize',1, 'linewidth', 1);
    plot([0 gripper_cone_left(1)], [0 gripper_cone_left(2)], '-.b', 'markersize',1, 'linewidth', 1);
    axis equal
    drawnow
end

% dir: direction of object rolling. (assume no sliding)
% try close direction first
if goal_pro(1) < 0
    dir_pro  = 1;
else
    dir_pro  = -1;
end

for rot_dir = 1:2
	
	% -----------------------------------------
	% 	Solve the 2D problem 
	% 	for object motion
	% -----------------------------------------

	[object_plan_2d_frames, init_roll_ang, flag] = plan2DObject(dir_pro, gravity_pro, init_pro, goal_pro, ...
						COMp_pro, gpp_pro, pointsp_pro, Rw_2d, pgraph.err_bound, gripper_cone_width, para);

	[object_plan_2d]  = check2DObjectTraj(object_plan_2d_frames, init_roll_ang, dir_pro, ...
	                              collision_free_range, cf_zero_id, gripper_cone_width);

	if isempty(object_plan_2d)
		plan_2d = [];
		if flag >= 0
			flag = -1;
		end
		dir_pro = - dir_pro;
		continue; % try the other direction
	end

	% Check 1: proper frame: move goal to edge of cone
	n                   = [0 0 1]';
	n                   = -n*dir_pro;
	objang              = sum(object_plan_2d.motion); 
	goal_final_pro      = quatOnVec(goal_pro, aa2quat(objang, n));
	assert(abs(goal_final_pro(3)) < 1e-5);
	gravity_pro_proj    = gravity_pro;
	gravity_pro_proj(3) = 0;
    assert(abs(angBTVec(-gravity_pro_proj, goal_final_pro, n) + object_plan_2d.ang2edge - gripper_cone_width) < 1e-4);
% 	assert(abs(pi - angBTVec(goal_grasp_2d(1:2), gravity_2d(1:2)) + object_plan_2d.ang2edge - gripper_cone_width) < 1e-4);

	% Check 2: p frame: move goal to edge of cone
	n_P          = gp1p_P - gp2p_P; n_P = n_P/norm(n_P);
	n_P          = -n_P*dir_pro;
	goal_final_p = quatOnVec(graspfpz_P, aa2quat(objang, n_P));
	% assert(abs(pi - angBTVec(goal_grasp_p(2:3), gravity_P(2:3)) + object_plan_2d.ang2edge - gripper_cone_width) < 1e-4);
	gravity_p_proj    = gravity_P;
	gravity_p_proj(1) = 0;
    assert(abs(angBTVec(-gravity_p_proj, goal_final_p, n_P) + object_plan_2d.ang2edge - gripper_cone_width) < 1e-4);

	% check 3: world frame, rotate object by object_plan_2d.motion, goal is at gripper_cone, check their angle
	n_w          = gp1p_w - gp2p_w; n_w = n_w/norm(n_w);
	gripper_cone = quatOnVec(zP_w, aa2quat(-object_plan_2d.dir*gripper_cone_width, n_w));

	qgoal_rot_w = quatMTimes(aa2quat(objang, -n_w*object_plan_2d.dir), qgraspfp_w);
	goal_now    = quatOnVec([0 0 1]', qgoal_rot_w);
	assert(abs(angBTVec(gripper_cone, goal_now) - object_plan_2d.ang2edge) < 5e-3);

	% -----------------------------------------
	% 	Solve for gripper motion (2D)
	% -----------------------------------------
	plan_2d    = plan2DGripper(object_plan_2d, init_grasp_ang, gripper_cone_width);

	if isempty(plan_2d)
		flag    = -5;
		dir_pro = - dir_pro;
		continue; % try the other direction
	end

	% check 4: gripper relative rotation angle
	relative_motion_togo = angBTquat(qgrasp0p_w, qgraspfp_w);
	relative_motion_cal  = sum(plan_2d.grp_motion_diff) + plan_2d.dir*sum(plan_2d.obj_motion_diff);
	assert( abs(relative_motion_togo - abs(ang_pi(relative_motion_cal))) < 1e-3);

	% Check 5: world frame check
	qgrasp_now = qgrasp0p_w;
	q_now      = qp; 

	for ii = 1:length(plan_2d.grp_motion_diff)
		qgrasp_now = quatMTimes(aa2quat(plan_2d.grp_motion_diff(ii), n_w), qgrasp_now);
		q_now      = quatMTimes(aa2quat(plan_2d.obj_motion_diff(ii), -n_w*plan_2d.dir), q_now);
	end
	q_goal    = quatMTimes(quatInv(qp), qgraspfp_w);
	q_achieve = quatMTimes(quatInv(q_now), qgrasp_now);

	assert(angBTquat(q_goal, q_achieve) < 1e-3); 

	break;
end

if isempty(plan_2d)
	flag = -5;
	qp   = [];
	return;
end

plan_2d.q0       = q0;
plan_2d.qf       = qf;
plan_2d.qp       = qp;
plan_2d.grasp_id = grasp_id;
plan_2d.qgrp0    = qgrasp0_w;

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




