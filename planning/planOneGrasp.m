% do planning given:
% 	grasp_id: a specific grasp pos
% 	q0, qf: initial/final object pose
% 	qg0, qgf: initial/final grasp pose (could be empty)
% 	exactq: true if we want to attain qf exactly. false if we don't care yaw angle
function [plan, flag] = planOneGrasp(grasp_id, q0, qf, qg0, qgf, exactq)
global mesh grasps pgraph para
plan = []; 

% -----------------------------------------
% 	Get the data
% -----------------------------------------

% under world coordinate, calculate the grasp pos for object in q0, qf
gp1o_w = grasps.points(:, grasp_id, 1);
gp2o_w = grasps.points(:, grasp_id, 2);
points = pgraph.vertices; % decimated convex hull
com    = mesh.COM;
ps_err = pgraph.err_bound;
CONE = atan(para.MU);

% Collision between gripper and table
gp10   = quatOnVec(gp1o_w, q0);
gp20   = quatOnVec(gp2o_w, q0);
qfr0   = getProperGraspSimple(gp10, gp20);
gp1f   = quatOnVec(gp1o_w, qf);
gp2f   = quatOnVec(gp2o_w, qf);
qfrf   = getProperGraspSimple(gp1f, gp2f);
gp0_x = quatOnVec([1 0 0]', qfr0);
gp0_z = quatOnVec([0 0 1]', qfr0);
gpf_x = quatOnVec([1 0 0]', qfrf);
gpf_z = quatOnVec([0 0 1]', qfrf);
gp10_bottom = gp10 + gp0_x*para.FINGER_OPEN_SPACE - gp0_z*para.FINGER_RADIUS;
gp20_bottom = gp20 - gp0_x*para.FINGER_OPEN_SPACE - gp0_z*para.FINGER_RADIUS;
gp1f_bottom = gp1f + gpf_x*para.FINGER_OPEN_SPACE - gpf_z*para.FINGER_RADIUS;
gp2f_bottom = gp2f - gpf_x*para.FINGER_OPEN_SPACE - gpf_z*para.FINGER_RADIUS;
points0 = quatOnVec(points, q0);
pointsf = quatOnVec(points, qf);
tablez0 = min(points0(3,:));
tablezf = min(pointsf(3,:));

if any([gp10_bottom(3)-tablez0 gp20_bottom(3)-tablez0 gp1f_bottom(3)-tablezf gp2f_bottom(3)-tablezf]< para.GRIPPER_Z_LIMIT)
	flag = -1;
	return;
end

%  Collision between gripper and object
refz_o = quatOnVec([0 0 1]', grasps.ref_frame(:, grasp_id));
refx_o = quatOnVec([1 0 0]', grasps.ref_frame(:, grasp_id));
% Find out the slice of collision-free range
% get seed
% seed must be within [-pi, pi] around current z
[~, z0_id]  = getIDinCfRange(qfr0, q0, refz_o, refx_o);
z0_id_range = (z0_id-90):(z0_id+90);
cf_range0   = circQuery(grasps.range(:, grasp_id), z0_id_range)';
start1      = strfind([0,cf_range0==1],[0 1]);
end1        = strfind([cf_range0==1,0],[1 0]);
seeds       = round( (end1 + start1)/2);
seeds       = z0_id_range(seeds);
Ncf         = length(seeds);

collision_free_range = zeros(360, Ncf);
for i = 1:Ncf
    if seeds(i) > 360
        seeds(i) = seeds(i) - 360;
    elseif seeds(i) < 1
        seeds(i) = seeds(i) + 360;
    end
	collision_free_range(:, i) = singleOutSFRange(grasps.range(:, grasp_id), seeds(i));
	assert(circQuery(collision_free_range(:, i), seeds(i)) == 1);
end

% Check if both initial/final grasp exists 
% in the same collision free range
cf_feasible  = true(1, Ncf); % choice of collision free range

if ~isempty(qg0)
	[~, zg0_id] = getIDinCfRange(qg0, q0, refz_o, refx_o);
	cf_feasible = collision_free_range(zg0_id, :)';
end

if ~isempty(qgf)
	[~, zgf_id] = getIDinCfRange(qgf, qf, refz_o, refx_o);
	cf_feasible = (cf_feasible&collision_free_range(zgf_id, :))';
end

if ~any(cf_feasible)
	flag = -2;
	return;
end

% -----------------------------------------
% 	Optimization: find qobj trajectory
% -----------------------------------------
N    = para.opt_obj_N;
qobj = quatSlerp(q0, qf, (1:N)/N);

% % init x
% x            = (2*rand(3*N,1) - 1)*pi;

% global paraOpt_OBJ
% paraOpt_OBJ.delta_theta   = para.opt_obj_con_delta_theta; % rad
% paraOpt_OBJ.q0            = q0;
% paraOpt_OBJ.qf            = qf;
% paraOpt_OBJ.qf_inv        = quatInv(qf);
% paraOpt_OBJ.Gp1o          = gp1o_w;
% paraOpt_OBJ.Gp2o          = gp2o_w;
% paraOpt_OBJ.Gp_tilt_limit = para.GRIPPER_TILT_LIMIT;
% paraOpt_OBJ.cost_dq_k     = para.opt_obj_cost_k_dq;
% paraOpt_OBJ.cost_tilt_k   = para.opt_obj_cost_k_tilt;
% paraOpt_OBJ.exactqf_k     = exactq;


% snoptOPT.spc = which('../optimization/snoptfiles/snoptOPT_OBJ.spc');
% snspec ( snoptOPT.spc );
% snseti ('Major Iteration limit', 50);
% snset  ('Minimize');

% snprint('../optimization/snoptfiles/snoptOPT_OBJ.out');
% [~, Flow, Fupp, xlow, xupp] = FUNOPT_OBJ(x, paraOpt_OBJ);
% tic
% [x,F,INFO] = snopt(x,xlow,xupp,Flow,Fupp,'FUNOPT_OBJWrapper');
% toc
% snprint off; % Closes the file and empties the print buffer

% % check results
% [con_x, con_f] = checkConstraint(x, paraOpt_OBJ, @FUNOPT_OBJ, xlow, xupp, Flow, Fupp);

% if norm(con_x)+norm(con_f) > 0.1
% 	% infeaseble 
% 	flag = -5;
% 	return;
% end	

% % read results
% n = zeros(3, N);
% for i = 1:N
% 	n(1, i) = x((i-1)*3+1);
% 	n(2, i) = x((i-1)*3+2);
% 	n(3, i) = x((i-1)*3+3);
% end
% theta       = normByCol(n);
% id          = theta < 1e-8;
% temp        = ones(3,1)*(sin(theta)./theta);
% temp(:, id) = 0;
% qobj        = [cos(theta); temp.*n];

% -----------------------------------------
% 	Evaluate the trajectory
% -----------------------------------------
Ng = 100;
% resample object orientation
qobj = quatSquad(0:N-1, qobj, (0:Ng-1)/(Ng-1)*(N-1));

% for each frame, compute:
% 	pivotable
% 	gripper feasible range (tilting range)
% 	collision free range
rtype         = zeros(1, Ng); % 1: pivotable, 0: roll
obj_rotation  = zeros(1, Ng); % rotation of object measured in grasp frame
tilt_range    = zeros(2, Ng); % min (right), max (left)
cf_range      = zeros(2, Ng, Ncf); % min (right), max (left)
grasp_qframes = zeros(4, Ng); % grasp frame at each time step
stuck      = false(1, Ng);
gpz        = zeros(1, Ng); % z pos of gripper
gpxy_delta = zeros(2, Ng); % xy offset of gripper
% gp1           = zeros(3, Ng); % grasp position (includes offset)
% gp2           = zeros(3, Ng);

points_fr_old = points;
gp_fr_old     = zeros(3, 1);

for fr = 1:Ng
	qp   = qobj(:, fr);
	m_fr = quat2m(qp);
	% 
	% check tilt angle limit
	% 
	gripper_cone_width = getTiltedGripperCone(grasp_id, qp, para.GRIPPER_TILT_LIMIT);
	if isempty(gripper_cone_width)
	    flag = -3;
		return;
	end
	tilt_range(1, fr) = - gripper_cone_width;
	tilt_range(2, fr) =   gripper_cone_width;

	% rotate and compute:
	% 	points on the object
	%	grasp point
	% 	COM
	% 	contact point on the table
	points_fr = m_fr*points; 
	com_fr    = m_fr*com;
	gp1_fr    = m_fr*gp1o_w;
	gp2_fr    = m_fr*gp2o_w;

	[zoffset, min_id] = min(points_fr(3,:));
	points_fr(3,:)    = points_fr(3,:) - zoffset;
	gp1_fr(3)         = gp1_fr(3) - zoffset;
	gp2_fr(3)         = gp2_fr(3) - zoffset;
	% 
    % check z limit
    % 
	if (gp1_fr(3) < para.GRIPPER_Z_LIMIT + ps_err)||(gp2_fr(3) < para.GRIPPER_Z_LIMIT + ps_err)
		flag = -4;
		return;
	end
	% gp1(:, fr) = gp1_fr;
	% gp2(:, fr) = gp2_fr;

	% 
	% check collision free range
	% 
	qfr           = getProperGraspSimple(gp1_fr, gp2_fr);
	[z_ang, z_id] = getIDinCfRange(qfr, qp, refz_o, refx_o);
	z_id_range    = (z_id-90):(z_id+90);

	obj_rotation(fr)     = z_ang;
	grasp_qframes(:, fr) = qfr;

	for cf = 1:Ncf
		% check with each range
		if ~cf_feasible(cf)
			continue;
		end
		cf_range_fr = circQuery(collision_free_range(:, cf), z_id_range)';
		start1      = strfind([0,cf_range_fr==1],[0 1]);
		end1        = strfind([cf_range_fr==1,0],[1 0]);
		if isempty(start1)
			cf_feasible(cf) = false;
			continue;
		end
		if length(start1) == 2
			center1    = (start1(1) + end1(1))/2;
			center2    = (start1(2) + end1(2))/2;
			center1_id = z_id_range(round(center1));
			center2_id = z_id_range(round(center2));
			dist2seed1 = abs(ang_180(center1_id - seeds(cf)));
			dist2seed2 = abs(ang_180(center2_id - seeds(cf)));
			if dist2seed1 < dist2seed2
				start1 = start1(1);
				end1   = end1(1);
			else
				start1 = start1(2);
				end1   = end1(2);
			end
		end
		temp_id_range       = -90:90;
		cf_range(1, fr, cf) = temp_id_range(start1)*pi/180;
		cf_range(2, fr, cf) = temp_id_range(end1)*pi/180;
	end

	if ~any(cf_feasible)
		flag = -6;
		return;
	end
	% 
	% check pivotablity (Robustly)
	% 

	% measure in grasp frame
	qfr_inv = quatInv(qfr);
	gp1_GF  = quatOnVec(gp1_fr, qfr_inv);
	gp2_GF  = quatOnVec(gp2_fr, qfr_inv);
	assert(abs(gp1_GF(2)-gp2_GF(2))<1e-5);
	com_GF = quatOnVec(com_fr, qfr_inv);
	com_y  = com_GF(2) - gp1_GF(2);

	cpid  = points_fr(3, :) < 2*ps_err + 1e-4;
	cp_fr = points_fr(:, cpid); % potential contact points
	cp_GF = quatOnVec(cp_fr, qfr_inv);
	cp_l  = min(cp_GF(2, :)) - gp1_GF(2);
	cp_r  = max(cp_GF(2, :)) - gp1_GF(2);
	if (cp_l*com_y > 0) && (cp_r*com_y > 0)
		rtype(fr) = 1;
	end

	% 
	% Check if getting stuck 
	% 	(going down and within friction cone)
	% 
	gp_fr        = (gp1_fr + gp2_fr)/2;
	gpz(fr)      = gp_fr(3);
	
	if fr > 1
	    % if gpz(fr) < gpz(fr-1)
	        % going down
			[~, cpid] = min(points_fr(3, :)); % < 2*ps_err + 1e-4;
			cp        = points_fr(:, cpid);
			gp_cp     = gp_fr - cp;

			cones     = atan2(norm(gp_cp(1:2)), abs(gp_cp(3)));
	        if cones < CONE
				stuck(fr)           = true;
				cp_old              = points_fr_old(:, cpid);
				gp_cp_old           = gp_fr_old - cp_old;
				gpxy_delta(:, fr-1) = gp_cp(1:2) - gp_cp_old(1:2);
	        end
		% end
	else
		gp0 = gp_fr;
	end

	points_fr_old = points_fr;	
	gp_fr_old     = gp_fr;
end


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
	qg0z_o           = quatOnVec(qg0z, quatInv(grasp_qframes(:,1)));
	grp_ang0         = angBTVec([0 0 1]', qg0z_o, [1 0 0]'); % range: -pi ~ pi
	paraOpt_GRP.x0_k = 1;
end

if ~isempty(qgf)
	qgfz             = quatOnVec([0 0 1]', qgf);
	qgfz_o           = quatOnVec(qgfz, quatInv(grasp_qframes(:,end)));
	grp_angf         = angBTVec([0 0 1]', qgfz_o, [1 0 0]'); % range: -pi ~ pi
	paraOpt_GRP.xf_k = 1;
end

% check range
cf_range = cf_range(:,:,cf_feasible);
if ( sum(cf_feasible) > 1 )
    range_sum = sum(cf_range(2,:,:) - cf_range(1,:,:),2);
    [~, id_sum] = max(range_sum);
    cf_range = cf_range(:,:,id_sum);
end
xrange      = zeros(2, Ng);
xrange(1,:) = max(tilt_range(1,:), cf_range(1,:));
xrange(2,:) = min(tilt_range(2,:), cf_range(2,:));
if any(xrange(2,:) < xrange(1,:))
	flag = -6;
	return;
end
temp = 0.5*ones(1, Ng);
x    = temp.*xrange(1,:) + (1-temp).*xrange(2,:);
x    = x';

paraOpt_GRP.range        = xrange;
paraOpt_GRP.N            = Ng;
paraOpt_GRP.rtype        = rtype;
paraOpt_GRP.obj_rotation = obj_rotation;
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
con_x = zeros(Ng, 2);
for i = 1:length(x)
	if x(i) < xlow(i)
		con_x(i, 1) = xlow(i) - x(i);
	end

	if x(i) > xupp(i)
		con_x(i, 2) = x(i) - xupp(i);
	end
end

% plot the planning problem for grp
% figure(1); clf(1);hold on;
% plot([1:Ng], x,'- .b');
% plot(find(paraOpt_GRP.rtype~=0), x(paraOpt_GRP.rtype~=0),'og');
% plot([1:Ng], xrange(1,:), '-r');
% plot([1:Ng], xrange(2,:), '-r');
% plot([1:Ng], obj_rotation, '-y');
% plot(1, paraOpt_GRP.x0, '.k', 'markersize',10);
% plot(Ng, paraOpt_GRP.xf, '.k', 'markersize',10);

if norm(con_x)+norm(con_f) > 0.1
	% infeaseble 
	flag = -7;
	return;
end	

% 
% read results
% 
qgrp = zeros(4, Ng);
for i = 1:Ng
	ax         = quatOnVec([1; 0; 0], grasp_qframes(:,i));
	q_incre    = aa2quat(x(i),ax);
	qgrp(:, i) = quatMTimes(q_incre, grasp_qframes(:, i));
end

plan.N          = Ng;
plan.rtype      = rtype;
plan.stuck      = stuck;
plan.qobj       = qobj;
plan.qgrp       = qgrp;
plan.grasp_id   = grasp_id; % just for animation
% plan.gp1        = gp1; % just for animation
% plan.gp2        = gp2; % just for animation
plan.gp0        = gp0;
plan.gpz        = gpz;
plan.gpxy_delta = gpxy_delta;

flag = 1;

end





% set sf_range to all zero, except the zone of 1 that constains id
function sf_range = singleOutSFRange(sf_range, id)
N = length(sf_range);
ids = false(N,1);

for i = id:-1:id - 360
	if circQuery(sf_range, i) == 1
		ids = circQuery(ids, i, 1);
	else
		break;
	end
end

for i = id+1:id + 360
	if circQuery(sf_range,i) == 1
		ids = circQuery(ids, i, 1);
	else
		break;
	end
end

sf_range      = ids;

end


function [z0_ang, z0_id] = getIDinCfRange(qg, q, refz_o, refx_o)
	qgz    = quatOnVec([0 0 1]', qg);
	qgz_o  = quatOnVec(qgz, quatInv(q));
	z0_ang = angBTVec(refz_o, qgz_o, refx_o, 1);
	z0_id  = round(180/pi*z0_ang);
end
