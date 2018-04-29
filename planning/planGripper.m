function [plan, flag] = planGripper(obj_plan, grasp_id, qg0, qgf)
global mesh grasps pgraph para

plan = [];
% -----------------------------------------
% 	Optimization: find gripper trajectory
% 	Formulated as a QP
% -----------------------------------------

% get initial/final grasps
grp_ang0         = 0;
grp_angf         = 0;
paraOpt_GRP.x0_k = 0;
paraOpt_GRP.xf_k = 0;

if ~isempty(qg0)
	qg0z             = quatOnVec([0 0 1]', qg0);
	qg0z_o           = quatOnVec(qg0z, quatInv(obj_plan.grasp_qframes(:,1)));
	grp_ang0         = angBTVec([0 0 1]', qg0z_o, [1 0 0]'); % range: -pi ~ pi
	paraOpt_GRP.x0_k = 1;
end

if ~isempty(qgf)
	qgfz             = quatOnVec([0 0 1]', qgf);
	qgfz_o           = quatOnVec(qgfz, quatInv(obj_plan.grasp_qframes(:,end)));
	grp_angf         = angBTVec([0 0 1]', qgfz_o, [1 0 0]'); % range: -pi ~ pi
	paraOpt_GRP.xf_k = 1;
end

% check range
cf_range = obj_plan.cf_range(:,:,obj_plan.cf_feasible);
if ( sum(obj_plan.cf_feasible) > 1 )
	range_sum   = sum(cf_range(2,:,:) - cf_range(1,:,:),2);
	[~, id_sum] = max(range_sum);
	cf_range    = cf_range(:,:,id_sum);
end
xrange      = zeros(2, obj_plan.Nf);
xrange(1,:) = max(obj_plan.tilt_range(1,:), cf_range(1,:));
xrange(2,:) = min(obj_plan.tilt_range(2,:), cf_range(2,:));
if any(xrange(2,:) < xrange(1,:))
	flag = -1;
	dispC('  --- [Gripper] [q obj checking] no collision free path ');
	return;
end
temp = 0.5*ones(1, obj_plan.Nf);
x    = temp.*xrange(1,:) + (1-temp).*xrange(2,:);
x    = x';

paraOpt_GRP.range        = xrange;
paraOpt_GRP.N            = obj_plan.Nf;
paraOpt_GRP.rtype        = obj_plan.rtype;
paraOpt_GRP.obj_rotation = obj_plan.obj_rotation;
paraOpt_GRP.x0           = grp_ang0;
paraOpt_GRP.xf           = grp_angf;
paraOpt_GRP.Qreg         = 1e-3;

[Q, A, B, Flow, Fupp, xlow, xupp] = FUNOPT_GRP_Precomputation(x, paraOpt_GRP);
[iAfun, jAvar, A_s] = find([zeros(size(x'*Q)); A]);
[iGfun, jGvar]    = find(ones(size(Q*x)));
global paraQ
paraQ = Q;


snoptOPT.spc = which('../optimization/snoptfiles/snoptOPT_GRP.spc');
snspec ( snoptOPT.spc );
snseti ('Major Iteration limit', 50);
snset  ('Minimize');
snprint off;
% snprint('../optimization/snoptfiles/snoptOPT_GRP.out'); 
% tic
[x, F, INFO] = snopt(x,xlow,xupp,Flow,Fupp,'FUNOPT_GRP', ...
		   A_s, iAfun, jAvar, iGfun, jGvar);
% toc
% snprint off; % Closes the file and empties the print buffer

% check results
con_f = A*x - B;
con_x = zeros(obj_plan.Nf, 2);
for i = 1:length(x)
	if x(i) < xlow(i)
		con_x(i, 1) = xlow(i) - x(i);
	end

	if x(i) > xupp(i)
		con_x(i, 2) = x(i) - xupp(i);
	end
end

% % plot the planning problem for grp
% figure(1); clf(1);hold on;
% plot([1:obj_plan.Nf], x,'- .b');
% plot(find(paraOpt_GRP.rtype~=0), x(paraOpt_GRP.rtype~=0),'og');
% plot([1:obj_plan.Nf], xrange(1,:), '-r');
% plot([1:obj_plan.Nf], xrange(2,:), '-r');
% plot([1:obj_plan.Nf], obj_plan.obj_rotation, '-y');
% plot(1, paraOpt_GRP.x0, '.k', 'markersize',10);
% plot(obj_plan.Nf, paraOpt_GRP.xf, '.k', 'markersize',10);

if norm(con_x)+norm(con_f) > 0.1
	% infeaseble 
	flag = -2;
	dispC('  --- [Gripper] [QP for GRP] infeasible');
	return;
end	

% 
% read results
% 
qgrp = zeros(4, obj_plan.Nf);
for i = 1:obj_plan.Nf
	ax         = quatOnVec([1; 0; 0], obj_plan.grasp_qframes(:,i));
	q_incre    = aa2quat(x(i),ax);
	qgrp(:, i) = quatMTimes(q_incre, obj_plan.grasp_qframes(:, i));
end

plan.N          = obj_plan.Nf;
plan.rtype      = obj_plan.rtype;
plan.stuck      = obj_plan.stuck;
plan.qobj       = obj_plan.qobj;
plan.qgrp       = qgrp;
plan.grasp_id   = grasp_id; % just for animation
% plan.gp1        = gp1; % just for animation
% plan.gp2        = gp2; % just for animation
plan.gp0        = obj_plan.gp0;
plan.gpz        = obj_plan.gpz;
plan.gpxy_delta = obj_plan.gpxy_delta;

% % offset for the actual scene
% plan.gp0 = plan.gp0 + para.scene_offset;
% plan.gpz = plan.gpz + para.scene_offset[3];


flag = 1;