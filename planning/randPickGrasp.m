function [grasp_id, qgrb, gp1_w, gp2_w] = randPickGrasp(qobj)
global grasps mesh pgraph para
% check all grasps for current pose
grasp_id = [];
qgrb     = [];
gp1_w    = [];
gp2_w    = [];

all_grasps = checkGraspPoints(grasps, mesh, pgraph, qobj, para.showGraspChecking_id(2), para);
ids        = find(all_grasps);

if isempty(ids)
	dispC('No feasible grasps for this pose !!');
	return;
end

id_rand = randperm(length(ids));
for r = 1:length(ids)
	i      = id_rand(r);
	gp1o_w = grasps.points(:, ids(i), 1);
	gp2o_w = grasps.points(:, ids(i), 2);
	gp1_w  = quatOnVec(gp1o_w, qobj);
	gp2_w  = quatOnVec(gp2o_w, qobj);

	gripper_cone_width = getTiltedGripperCone(gp1_w, gp2_w, para.GRIPPER_TILT_LIMIT);
	qg0                = getProperGrasp(ids(i), qobj, gripper_cone_width, true);
	if isempty(qg0)
		continue;
	end
	grasp_id         = zeros(1, grasps.count);
	grasp_id(ids(i)) = 1;
	qgrb             = qg0;

	return;
end	

dispC('No feasible grasps for this pose !!');
