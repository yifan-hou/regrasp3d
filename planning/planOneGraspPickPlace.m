% do planning given:
% 	grasp_id: a specific grasp pos
% 	q0, qf: initial/final object pose
% 	qg0, qgf: initial/final grasp pose (could be empty)
function [plan_2d, flag] = planOneGraspPickPlace(grasp_id, q0, qf, qg0, qgf)
global grasps para
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

% ----------------------------------------
% 	Pick initial & final grasps
% 	Calculate gripper_cone_width for q0 and qf
% ----------------------------------------
gripper_cone_width0 = getTiltedGripperCone(grasp_id, q0, para.GRIPPER_TILT_LIMIT);
gripper_cone_widthf = getTiltedGripperCone(grasp_id, qf, para.GRIPPER_TILT_LIMIT);
if isempty(gripper_cone_width0)
    flag = -3;
	return;
end
if isempty(gripper_cone_widthf)
    flag = -3;
	return;
end

% % check grasp frame
% temp  = gp10_w - gp20_w;
% tempv = quatOnVec([1 0 0]', qgrasp0_w);
% assert( abs(angBTVec(temp, tempv)) < 1e-7);


% -----------------------------------
%  Find a common grasp
% -----------------------------------

% get range
[~, ~, data] = getProperGrasp(grasp_id, q0, gripper_cone_width0, false);
id_center    = data.id_center;
range0       = data.range;
qg0frame     = data.qg0;
if ~isempty(qg0)
	v  	   = gp10_w - gp20_w; v  = v/norm(v);
	z0     = quatOnVec([0 0 1]', qg0frame);
	z1     = quatOnVec([0 0 1]', qg0);
	angqg0 = round(angBTVec(z0, z1, v, 1)*180/pi); % 0~360

	angqg0_zone = circQuery(range0, angqg0);
	range0      = circQuery(0*range0, angqg0, angqg0_zone);
end

[~, ~, data] = getProperGrasp(grasp_id, qf, gripper_cone_widthf, false);
rangef       = data.range;
qgfframe     = data.qg0;
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
% qg_ref    = getProperGraspSimple(gp1o_w, gp2o_w);
qgrasp0_w = quatMTimes(aa2quat(ang, gp1o_w-gp2o_w), grasps.ref_frame(:, grasp_id));
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