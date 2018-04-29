% gp: grasp points in world frame
% q: 4x1
function [] = generateRobotTraj(plan)
% global mesh 

clc;
% --------------------------------------
% 	Robot coordinate frames
% --------------------------------------
trans_p2r  = para.scene_offset;
compensate = 180-22.5;
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

N           = length(plan);
plan_smooth = cell(N,1);
for p = 1:N
	temp_rtype      = [];
	temp_stuck      = [];
	temp_qgp        = [];
	temp_gpz        = [];
	temp_gpxy_delta = [];

    for fr = 1:plan{p}.N-1
        qgrp_seg = plan{p}.qgrp(:, fr:fr+1);
        grpz_seg = plan{p}.gpz(:, fr:fr+1);
        
        Max_ang  = angBTquat(qgrp_seg(:,1), qgrp_seg(:,2));
        Max_dist = abs(grpz_seg(1) - grpz_seg(2));
        
        K = max(Max_ang/max_diff_q, Max_dist/max_diff_p);
        
        n_seg_new = round(K*2);
        if n_seg_new < 2
            n_seg_new = 2;
        end
        
		middle_qgp        = quatSlerp(plan{p}.qgrp(:, fr), plan{p}.qgrp(:, fr+1), (0:n_seg_new-1)/(n_seg_new-1));
		middle_gpz        = interp1([0 1], plan{p}.gpz(:, fr:fr+1), (0:n_seg_new-1)/(n_seg_new-1));
		middle_gpxy_delta = plan{p}.gpxy_delta(:, fr)*ones(1, n_seg_new)/n_seg_new;
		temp_qgp          = [temp_qgp middle_qgp];
		temp_gpz          = [temp_gpz middle_gpz];
		temp_gpxy_delta   = [temp_gpxy_delta middle_gpxy_delta];
        
        if (plan{p}.rtype(fr) == false) && (plan{p}.rtype(fr+1) == false)
            temp_rtype = [temp_rtype false(1, n_seg_new)];
        else
            temp_rtype = [temp_rtype true(1, n_seg_new)];
        end

        if (plan{p}.stuck(fr) == false) && (plan{p}.stuck(fr+1) == false)
            temp_stuck = [temp_stuck false(1, n_seg_new)];
        else
            temp_stuck = [temp_stuck true(1, n_seg_new)];
        end
    end

	plan_smooth{p}.N          = length(temp_rtype);
	plan_smooth{p}.rtype      = temp_rtype;
	plan_smooth{p}.stuck      = temp_stuck;
	plan_smooth{p}.qgrp       = temp_qgp;
	plan_smooth{p}.gpz        = temp_gpz;
	plan_smooth{p}.gpxy_delta = temp_gpxy_delta;
end

% --------------------------------------
% 	Begin Writting to file
% --------------------------------------
N = length(plan_smooth);
rmdir('../results/traj*','s');
for i=1:N
	mkdir(['../results/traj' num2str(i)]);
end


for p = 1:N
	f_N           = fopen(['../results/traj' num2str(p) '/N.txt'],'w');
	f_rtype       = fopen(['../results/traj' num2str(p) '/rtype.txt'],'w');
	f_stuck       = fopen(['../results/traj' num2str(p) '/stuck.txt'],'w');
	f_qgrp        = fopen(['../results/traj' num2str(p) '/qgrp.txt'],'w');
	f_grp0        = fopen(['../results/traj' num2str(p) '/grp0.txt'],'w');
	f_grpz        = fopen(['../results/traj' num2str(p) '/grpz.txt'],'w');
	f_grpxy_delta = fopen(['../results/traj' num2str(p) '/grpxy_delta.txt'],'w');
	for fr = 1:plan_smooth{p}.N
		% read
		rtype      = plan_smooth{p}.rtype(fr);
		stuck      = plan_smooth{p}.stuck(fr);
		qg_p       = plan_smooth{p}.qgrp(:, fr); % grasp frame, measured in planning
		gpz        = plan_smooth{p}.gpz(:, fr);
		gpxy_delta = plan_smooth{p}.gpxy_delta(:, fr);

		% transform
		zg    = quatOnVec([0 0 1]', qg_p);
		xg    = quatOnVec([1 0 0]', qg_p);
		qcp1  = aa2quat(compensate*pi/180, zg);
		xw    = quatOnVec(xg, qcp1);
		q_g2t = quatMTimes(aa2quat(pi, xw), qcp1); % transition from grasp frame to robot tool frame 

		qt_p     = quatMTimes(q_g2t, qg_p); % tool frame measured in planning
		qt_r     = qt_p; % tool frame measured in robot frame (done)
		% transg_r = transg_p + trans_p2r;
		% transt_r = transg_r;
		toolz        = gpz + trans_p2r(3);
		toolxy_delta = gpxy_delta;

		% print
		fprintf(f_rtype, '%d', rtype);
		fprintf(f_stuck, '%d', stuck);
		fprintf(f_qgrp, '%f\t%f\t%f\t%f', qt_r(1), qt_r(2), qt_r(3), qt_r(4));
		fprintf(f_grpz, '%f', toolz);
		fprintf(f_grpxy_delta, '%f\t%f', toolxy_delta(1), toolxy_delta(2));
		if fr ~= plan_smooth{p}.N
			fprintf(f_rtype, '\n');
			fprintf(f_stuck, '\n');
			fprintf(f_qgrp, '\n');
			fprintf(f_grpz, '\n');
			fprintf(f_grpxy_delta, '\n');
		end
	end
	gp0 = plan{p}.gp0 + trans_p2r;
	fprintf(f_N, '%d\n', plan_smooth{p}.N);
	fprintf(f_grp0, '%f\t%f\t%f\n', gp0(1), gp0(2), gp0(3));
	
	fclose(f_N);
	fclose(f_rtype);
	fclose(f_stuck);
	fclose(f_qgrp);
	fclose(f_grp0);
	fclose(f_grpz);
	fclose(f_grpxy_delta);
end

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