function [plan, flag] = planGripper(obj_plan, grasp_id, qg0, qgf)

plan = [];
% -----------------------------------------
% 	Optimization: find gripper trajectory
% 	Formulated as a QP
% -----------------------------------------
for gps = 1:length(grasp_id)
	% get initial/final grasps
	grp_ang0         = 0;
	grp_angf         = 0;
	paraOpt_GRP.x0_k = 0;
	paraOpt_GRP.xf_k = 0;

	if ~isempty(qg0)
		q_WGz_initial    = quatOnVec([0 0 1]', qg0);
		q_OGz_initial    = quatOnVec(q_WGz_initial, quatInv(obj_plan{gps}.qs_WG_proper(:,1)));
		grp_ang0         = angBTVec([0 0 1]', q_OGz_initial, [1 0 0]'); % range: -pi ~ pi
		paraOpt_GRP.x0_k = 1;
	end

	if ~isempty(qgf)
		qgfz             = quatOnVec([0 0 1]', qgf);
		qgfz_o           = quatOnVec(qgfz, quatInv(obj_plan{gps}.qs_WG_proper(:,end)));
		grp_angf         = angBTVec([0 0 1]', qgfz_o, [1 0 0]'); % range: -pi ~ pi
		paraOpt_GRP.xf_k = 1;
	end

	% check range
	ns_collision_free_range = obj_plan{gps}.ns_collision_free_range(:,:,obj_plan{gps}.bs_collision_free_gripper_angles);
	if ( sum(obj_plan{gps}.bs_collision_free_gripper_angles) > 1 )
		range_sum   = sum(ns_collision_free_range(2,:,:) - ns_collision_free_range(1,:,:),2);
		[~, id_sum] = max(range_sum);
		ns_collision_free_range    = ns_collision_free_range(:,:,id_sum);
	end
	xrange      = zeros(2, obj_plan{gps}.kNumOfFrames);
	xrange(1,:) = max(obj_plan{gps}.ns_gripper_tilt_ranges(1,:), ns_collision_free_range(1,:));
	xrange(2,:) = min(obj_plan{gps}.ns_gripper_tilt_ranges(2,:), ns_collision_free_range(2,:));
	if any(xrange(2,:) < xrange(1,:))
		% flag = -1;
		dispC(['  --- [Gripper] #' num2str(gps) ' [q obj checking] no collision free path ']);
		continue;
	end
	temp = 0.5*ones(1, obj_plan{gps}.kNumOfFrames);
	x    = temp.*xrange(1,:) + (1-temp).*xrange(2,:);
	x    = x';

	paraOpt_GRP.range        = xrange;
	paraOpt_GRP.N            = obj_plan{gps}.kNumOfFrames;
	paraOpt_GRP.rtype        = obj_plan{gps}.b_pivoting;
	paraOpt_GRP.obj_rotation = obj_plan{gps}.ns_obj_angle_in_G_proper;
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
	con_x = zeros(obj_plan{gps}.kNumOfFrames, 2);
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
	% plot([1:obj_plan{gps}.kNumOfFrames], x,'- .b');
	% plot(find(paraOpt_GRP.rtype~=0), x(paraOpt_GRP.rtype~=0),'og');
	% plot([1:obj_plan{gps}.kNumOfFrames], xrange(1,:), '-r');
	% plot([1:obj_plan{gps}.kNumOfFrames], xrange(2,:), '-r');
	% plot([1:obj_plan{gps}.kNumOfFrames], obj_plan{gps}.ns_obj_angle_in_G_proper, '-y');
	% plot(1, paraOpt_GRP.x0, '.k', 'markersize',10);
	% plot(obj_plan{gps}.kNumOfFrames, paraOpt_GRP.xf, '.k', 'markersize',10);

	if norm(con_x)+norm(con_f) > 0.01
		% infeaseble
		% flag = -2;
		dispC(['  --- [Gripper] #' num2str(gps) ' [QP for GRP] infeasible']);
		continue;
	end

	%
	% read results
	%
	qgrp = zeros(4, obj_plan{gps}.kNumOfFrames);
	for i = 1:obj_plan{gps}.kNumOfFrames
		ax         = quatOnVec([1; 0; 0], obj_plan{gps}.qs_WG_proper(:,i));
		q_relative_to_center    = aa2quat(x(i),ax);
		qgrp(:, i) = quatMTimes(q_relative_to_center, obj_plan{gps}.qs_WG_proper(:, i));
	end

	plan.N          = obj_plan{gps}.kNumOfFrames;
	plan.rtype      = obj_plan{gps}.b_pivoting;
	plan.stuck      = obj_plan{gps}.b_is_stuck;
	plan.qobj       = obj_plan{gps}.qs_WO;
	plan.qgrp       = qgrp;
	plan.grasp_id   = grasp_id(gps); % just for animation
	% plan.gp1        = gp1; % just for animation
	% plan.gp2        = gp2; % just for animation
	plan.gp0        = obj_plan{gps}.p_WG_initial;
	plan.gpz        = obj_plan{gps}.n_WG_z;
	plan.gpxy_delta = obj_plan{gps}.v2_WG_xy_delta;
	plan.gp1o_w     = obj_plan{gps}.p_OG_left;
	plan.gp2o_w     = obj_plan{gps}.p_OG_right;


	flag = 1;
	return;
end

% no solution
flag = -2;
dispC('  --- [Gripper] infeasible');