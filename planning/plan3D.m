% input:
% 	plan_2d: a cell array of structures
% explanation of key variables
% 	gpc_xy: the final actual xy position of grasp center
% 	xy_offset: offset from q*p0 to true position

function plan_3d = plan3D(plan_2d)
% global para fgraph pgraph mesh gripper grasps
global mesh grasps

STEP_LENGTH_RAD   = 3*pi/180;
STEP_LENGTH_METER = 0.05;

plan_3d.qobj  = {};
plan_3d.qgrp  = {};
plan_3d.gp1   = {};
plan_3d.gp2   = {};
plan_3d.trans = {};
plan_3d.rtype = {};
plan_3d.stype = {};

xy_offset = [0; 0];

for p = 1:length(plan_2d)
	q0 = plan_2d{p}.q0;
	qf = plan_2d{p}.qf;
	qp = plan_2d{p}.qp;

	% grasp
	gp1o_w    = grasps.points(:, plan_2d{p}.grasp_id, 1);
	gp2o_w    = grasps.points(:, plan_2d{p}.grasp_id, 2);
	gp10_w    = quatOnVec(gp1o_w, q0);
	gp20_w    = quatOnVec(gp2o_w, q0);
	qgrasp0_w = plan_2d{p}.qgrp0;
	% getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate
	gpc_xy    = (gp10_w(1:2) + gp20_w(1:2))/2 + xy_offset; % only x,y are useful

	% --------------------------------------
	% 	Pre-Rolling to qp
	% --------------------------------------
	num = floor(angBTquat(q0, qp)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(q0, qp, t);

	qobj  = qi;
	qgrp  = zeros(size(qobj));
	gp1   = zeros(3, size(qobj, 2));
	gp2   = zeros(3, size(qobj, 2));
	trans = zeros(3, size(qobj, 2));
	rtype = false(1, size(qobj, 2)); % rolling
	stype = true(1, size(qobj, 2)); % sliding

	qgrp_now = qgrasp0_w;
	for s = 1:length(t)
		qobj_incre = quatMTimes(qi(:,s), quatInv(q0)); % a rotation that rotates q0 to qi(:,s), measured in world frame
		qgrp(:,s)  = quatMTimes(qobj_incre, qgrp_now);

		% cal grasp pos
		gp10_w    = quatOnVec(gp1o_w, qobj(:,s));
		gp20_w    = quatOnVec(gp2o_w, qobj(:,s));
		% z
		m0_o      = quat2m(qobj(:,s));
		points    = m0_o*(mesh.vertices');
		z_offset  = min(points(3,:));
		gp10_w(3) = gp10_w(3) - z_offset;
		gp20_w(3) = gp20_w(3) - z_offset;
		% x,y
		if norm(gpc_xy) < STEP_LENGTH_METER
			gpc_xy = [0; 0];
		else
			gpc_xy = gpc_xy - gpc_xy/norm(gpc_xy)*STEP_LENGTH_METER;
		end
		xy_offset   = gpc_xy - (gp10_w(1:2)+gp20_w(1:2))/2;
		gp10_w(1:2) = gp10_w(1:2) + xy_offset;
		gp20_w(1:2) = gp20_w(1:2) + xy_offset;
		gp1(:, s)   = gp10_w;
		gp2(:, s)   = gp20_w;
		trans(:, s) = [xy_offset; -z_offset];
	end

	plan_3d.qobj{p}  = qobj;
	plan_3d.qgrp{p}  = qgrp;
	plan_3d.gp1{p}   = gp1;
	plan_3d.gp2{p}   = gp2;
	plan_3d.trans{p} = trans;
	plan_3d.rtype{p} = rtype;
	plan_3d.stype{p} = stype;

	% update states
	gp10_w = quatOnVec(gp10_w, qobj_incre);
	gp20_w = quatOnVec(gp20_w, qobj_incre);
    
	% --------------------------------------
	% 	Pivoting
	% --------------------------------------
	ang_obj = 0;
	ang_grp = 0;
	n       = gp10_w - gp20_w; n = n/norm(n); % rotation axis
    
    qobj_now = qp;
    qgrp_now = qgrp(:, end);

    % get bottom id&pos
    [~, cp_id_old] = min(points(3,:));
    cp_old = points(1:2, cp_id_old) + xy_offset;

	for s = 1:length(plan_2d{p}.grp_motion_diff)
		ang_obj_incre = plan_2d{p}.obj_motion_diff(s);
		ang_grp_incre = plan_2d{p}.grp_motion_diff(s);
		assert(~any(ang_obj_incre < 0)); 

		num = floor(max(abs(ang_obj_incre), abs(ang_grp_incre))/STEP_LENGTH_RAD);
		if num == 0
			num = 1;
		end

		ang_obj_incre_array = fixStepSample(0, ang_obj_incre, num); % length: num+1
		ang_grp_incre_array = fixStepSample(0, ang_grp_incre, num);

		qobj  = zeros(4, num+1);
		qgrp  = zeros(4, num+1);
		gp1   = zeros(3, num+1);
		gp2   = zeros(3, num+1);
		trans = zeros(3, num+1);
		rtype = true(1, num+1);
		stype = false(1, num+1);

		for i = 1:num+1
			% calculate
			qobj_incre = aa2quat(ang_obj + ang_obj_incre_array(i), -n*plan_2d{p}.dir);
			qgrp_incre = aa2quat(ang_grp + ang_grp_incre_array(i), n);

			qobj(:,i) = quatMTimes(qobj_incre, qobj_now);
			qgrp(:,i) = quatMTimes(qgrp_incre, qgrp_now);

			% cal grasp pos
			gp10_w = quatOnVec(gp1o_w, qobj(:,i));
			gp20_w = quatOnVec(gp2o_w, qobj(:,i));
			% z
			m0_o                  = quat2m(qobj(:,i));
			points                = m0_o*(mesh.vertices');
			[z_offset, cp_id_now] = min(points(3,:));
			gp10_w(3)             = gp10_w(3) - z_offset;
			gp20_w(3)             = gp20_w(3) - z_offset;
			% x,y
			% check sliding type
			id       = getIDinArray(plan_2d{p}.obj_sliding_acc, ang_obj + ang_obj_incre_array(i));
			stype(i) = plan_2d{p}.obj_sliding_type(id);
			rtype(i) = plan_2d{p}.rtype(s);

			if stype(i) == true
				% sliding is ok
				% move gpc towards [0 0]
				if norm(gpc_xy) < STEP_LENGTH_METER
					gpc_xy = [0; 0];
				else
					gpc_xy = gpc_xy - gpc_xy/norm(gpc_xy)*STEP_LENGTH_METER;
				end
				xy_offset = gpc_xy - (gp10_w(1:2)+gp20_w(1:2))/2;
			else
				% has to pivot about contact point
				% move the last contact point back
				cp_now    = points(1:2, cp_id_old);
				xy_offset = cp_old - cp_now;
				gpc_xy    = xy_offset + (gp10_w(1:2)+gp20_w(1:2))/2;
			end
			cp_id_old = cp_id_now;
			cp_old    = points(1:2, cp_id_now) + xy_offset;

			gp10_w(1:2) = gp10_w(1:2) + xy_offset;
			gp20_w(1:2) = gp20_w(1:2) + xy_offset;
			gp1(:, i)   = gp10_w;
			gp2(:, i)   = gp20_w;
			trans(:, i) = [xy_offset; -z_offset];
		end
		ang_obj = ang_obj + ang_obj_incre;
		ang_grp = ang_grp + ang_grp_incre;

		plan_3d.qobj{p}  = [plan_3d.qobj{p} qobj];
		plan_3d.qgrp{p}  = [plan_3d.qgrp{p} qgrp];
		plan_3d.gp1{p}   = [plan_3d.gp1{p}  gp1];
		plan_3d.gp2{p}   = [plan_3d.gp2{p}  gp2];
		plan_3d.trans{p} = [plan_3d.trans{p}  trans];
		plan_3d.rtype{p} = [plan_3d.rtype{p}  rtype];
		plan_3d.stype{p} = [plan_3d.stype{p}  stype];
	end

	% --------------------------------------
	% 	Post-Rolling
    %   From qobj_now to qf
	% --------------------------------------
    qobj_now = qobj(:, end);
    qgrp_now = qgrp(:, end);
	num = floor(angBTquat(qobj_now, qf)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(qobj_now, qf, t);
	
	qobj  = qi;
	qgrp  = zeros(size(qobj));
	gp1   = zeros(3, size(qobj, 2));
	gp2   = zeros(3, size(qobj, 2));
	trans = zeros(3, size(qobj, 2));
	rtype = false(1, size(qobj, 2)); % rolling
	stype = true(1, size(qobj, 2)); % sliding


	for s = 1:length(t)
		qobj_incre = quatMTimes(qi(:,s), quatInv(qobj_now)); % a rotation that rotates qobj_now to qi(:,s), measured in world frame
		qgrp(:,s)  = quatMTimes(qobj_incre, qgrp_now);

		% cal grasp pos
		gp10_w    = quatOnVec(gp1o_w, qobj(:,s));
		gp20_w    = quatOnVec(gp2o_w, qobj(:,s));

		% z
		m0_o      = quat2m(qobj(:,s));
		points    = m0_o*(mesh.vertices');
		z_offset  = min(points(3,:));
		gp10_w(3) = gp10_w(3) - z_offset;
		gp20_w(3) = gp20_w(3) - z_offset;
		% x,y
		if norm(gpc_xy) < STEP_LENGTH_METER
			gpc_xy = [0; 0];
		else
			gpc_xy = gpc_xy - gpc_xy/norm(gpc_xy)*STEP_LENGTH_METER;
		end
		xy_offset   = gpc_xy - (gp10_w(1:2)+gp20_w(1:2))/2;
		gp10_w(1:2) = gp10_w(1:2) + xy_offset;
		gp20_w(1:2) = gp20_w(1:2) + xy_offset;
		gp1(:, s)   = gp10_w;
		gp2(:, s)   = gp20_w;
		trans(:, s) = [xy_offset; -z_offset];
	end

	plan_3d.qobj{p}  = [plan_3d.qobj{p}   qobj];
	plan_3d.qgrp{p}  = [plan_3d.qgrp{p}   qgrp];
	plan_3d.gp1{p}   = [plan_3d.gp1{p}    gp1];
	plan_3d.gp2{p}   = [plan_3d.gp2{p}    gp2];
	plan_3d.trans{p} = [plan_3d.trans{p}    trans];
	plan_3d.rtype{p} = [plan_3d.rtype{p}  rtype];
	plan_3d.stype{p} = [plan_3d.stype{p}  stype];
end % finish the whole path


end


% function id = getIDinArray(array, a, b)
% 	i1 = 1;
% 	while array(i1)<a
% 		i1 = i1+1;
% 	end
% 	i2 = 1;
% 	while array(i2)<b
% 		i2 = i2+1;
% 	end
% 	id = id1:id2;
% end

function id1 = getIDinArray(array, a)
	id1 = 1;
	while array(id1)<a
		id1 = id1+1;
	end
end