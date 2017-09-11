function score = evalPlan(plan_3d)

if isempty(plan_3d)
	score = [];
	return;
end

score.gripper_rotation    = 0;
score.gripper_translation = 0;
score.execution_time      = 0;
qgrp                      = [1 0 0 0]';
gpc                       = [0 0 1]';
NP                        = length(plan_3d.qobj);
% number of regrasps
score.N_regrasp = NP;
for p = 1:NP
	% execution time
	Nfr                  = size(plan_3d.qobj{p}, 2);
	score.execution_time = score.execution_time + Nfr;
	for fr = 1:Nfr
		% total gripper rotation
		qgrp_new               = plan_3d.qgrp{p}(:, fr);
		score.gripper_rotation = score.gripper_rotation + angBTquat(qgrp, qgrp_new);
		qgrp                   = qgrp_new;

		% total gripper translation
		gp1                       = plan_3d.gp1{p}(:, fr);
		gp2                       = plan_3d.gp2{p}(:, fr);
		gpc_new                   = (gp1 + gp2)/2;
		score.gripper_translation = score.gripper_translation + norm(gpc - gpc_new);
		gpc                       = gpc_new;
		
	end
end

end