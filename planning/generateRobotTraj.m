% gp: grasp points in world frame
% q: 4x1
function generateRobotTraj(plan)
% global mesh 

clc;
% --------------------------------------
% 	Robot coordinate frames
% --------------------------------------
trans_p2r = [17.37 -474.66 5]';
compensate = 36;

Velocity_p = 100; % mm/s
Velocity_q = 15*pi/180; % mm/s
Rate       = 500; %Hz

% --------------------------------------
% 	draw and get the handles
% --------------------------------------
% object states
% q0        = plan{1}.qobj(:,1);
% m0_o      = quat2m(q0);
% com_w     = m0_o*mesh.COM;
% points_w  = m0_o*(mesh.vertices');

% Initial plotting, get handles,
% contact points
% [~, cpid] = min(points_w(3,:)); 
% cpid      = abs(points_w(3,:) - points_w(3,cpid))<1e-5;
% cp_w      = points_w(:,cpid);


% --------------------------------------
% 	Interpolation
% --------------------------------------
max_diff_p = Velocity_p/Rate;
max_diff_q = Velocity_q/Rate;

N = length(plan);
plan_smooth = cell(N,1);
assert(N==1); % for testing only
for p = 1:N
	temp_rtype = [];
	temp_qgp   = [];
	temp_gp    = [];

    plan{p}.pgrp = (plan{p}.gp1 + plan{p}.gp2)/2;
    for fr = 1:plan{p}.N-1
        qgrp_seg = plan{p}.qgrp(:, fr:fr+1);
        pgrp_seg = plan{p}.pgrp(:, fr:fr+1);
        
        Max_ang  = findMaxAngBtQuat(qgrp_seg);
        Max_dist = findMaxDistBtP(pgrp_seg);
        
        K = max(Max_ang/max_diff_q, Max_dist/max_diff_p);
        
        n_seg_new = round(K*2);
        if n_seg_new < 2
            n_seg_new = 2;
        end
        
        middle_qgp = quatSlerp(plan{p}.qgrp(:, fr), plan{p}.qgrp(:, fr+1), (0:n_seg_new-1)/(n_seg_new-1));
        middle_gp  = interp1([0 1], plan{p}.pgrp(:, fr:fr+1)', (0:n_seg_new-1)/(n_seg_new-1))';
        temp_qgp   = [temp_qgp middle_qgp];
        temp_gp    = [temp_gp middle_gp];
        
        if (plan{p}.rtype(fr) == 0) && (plan{p}.rtype(fr+1)==0)
            temp_rtype = [temp_rtype false(1, n_seg_new)];
        else
            temp_rtype = [temp_rtype true(1, n_seg_new)];
        end
    end

	plan_smooth{p}.rtype = temp_rtype;
	plan_smooth{p}.qgrp  = temp_qgp;
	plan_smooth{p}.gp    = temp_gp;
	plan_smooth{p}.N     = length(temp_rtype);
end

% --------------------------------------
% 	Begin Writting to file
% --------------------------------------
f_rtype = fopen('../results/rtype.txt','w');
f_qgrp  = fopen('../results/qgrp.txt','w');
f_pgrp  = fopen('../results/pgrp.txt','w');
f_N  = fopen('../results/N.txt','w');

N = length(plan_smooth);
assert(N==1); % for testing only
for p = 1:N
	for fr = 1:plan_smooth{p}.N
		% read
		% qobj  = plan_smooth{p}.qobj(:, fr);
		rtype    = plan_smooth{p}.rtype(fr);
		qg_p     = plan_smooth{p}.qgrp(:, fr);
		gp       = plan_smooth{p}.gp(:, fr);
		% transg_p = plan_smooth{p}.trans(:, fr);

		% transform
		zg    = quatOnVec([0 0 1]', qg_p);
		xg    = quatOnVec([1 0 0]', qg_p);
		qcp1  = aa2quat(compensate*pi/180, zg);
		xw    = quatOnVec(xg, qcp1);
		q_g2t = quatMTimes(aa2quat(pi, xw), qcp1);

		qt_p     = quatMTimes(q_g2t, qg_p);
		qt_r     = qt_p;
		% transg_r = transg_p + trans_p2r;
		% transt_r = transg_r;
		tool     = gp + trans_p2r;

		% print
		fprintf(f_rtype, '%d', rtype);
		fprintf(f_qgrp, '%f\t%f\t%f\t%f', qt_r(1), qt_r(2), qt_r(3), qt_r(4));
		fprintf(f_pgrp, '%f\t%f\t%f', tool(1), tool(2), tool(3));
		if fr ~= plan_smooth{p}.N
			fprintf(f_rtype, '\n');
			fprintf(f_qgrp, '\n');
			fprintf(f_pgrp, '\n');
		end
	end
	fprintf(f_N, '%d', plan_smooth{p}.N);
end
fclose(f_rtype);
fclose(f_qgrp);
fclose(f_pgrp);
fclose(f_N);

disp('Trajectory is written to file.');
end



function ang = findMaxAngBtQuat(q)
	N = size(q,2);
	ang = 0;
	for i = 1:N-1
		a = angBTquat(q(:,i), q(:,i+1));
		if a > ang
			ang = a;
		end
	end
end


function dist = findMaxDistBtP(p)
	N = size(p,2);
	dist = 0;
	for i = 1:N-1
		a = norm(p(:,i)-p(:,i+1));
		if a > dist
			dist = a;
		end
	end
end