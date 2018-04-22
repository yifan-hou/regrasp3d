% plan object motion, select a grasp given:
% 	q0, qf: initial/final object pose
% 	qg0, qgf: initial/final grasp pose (could be empty)
% 	exactq: true if we want to attain qf exactly. false if we don't care yaw angle
%   method: 'pickplace' or 'pivoting'
function [obj_plan, id_sel] = planObject(grasp_ids, q0, qf, qg0, qgf, exactq, method)

global mesh grasps pgraph para
obj_plan = []; 

Ng = length(grasp_ids);

points  = pgraph.vertices; % decimated convex hull
com     = mesh.COM;
ps_err  = pgraph.err_bound;
CONE    = atan(para.MU);
points0 = quatOnVec(points, q0);
pointsf = quatOnVec(points, qf);
tablez0 = min(points0(3,:));
tablezf = min(pointsf(3,:));

% grasps flag:
% 	  1: successful
% 	  -1: [Pre-checking] initial/final gripper collides with table;
%     -2: [Pre-checking] initial/final grasp infeasible in cf_range
%     -3: [q obj optimization] violates gripper tilt angle limit 
%     -4: [q obj optimization] violates gripper Z limit 
%     -5: [q obj optimization] infeasible 
%     -6: [q obj checking] no collision-free path 
grasps_flag  = ones(Ng, 1);

Nf   = 50;
qobj = quatSlerp(q0, qf, (1:Nf)/Nf);

% qobj = quatSquad(0:N-1, qobj, (0:Nf-1)/(Nf-1)*(N-1));

cf_feasible   = cell(Ng, 1);
rtype         = cell(Ng, 1);
obj_rotation  = cell(Ng, 1);
tilt_range    = cell(Ng, 1);
cf_range      = cell(Ng, 1);
grasp_qframes = cell(Ng, 1);
stuck         = cell(Ng, 1);
gpz           = cell(Ng, 1);
gpxy_delta    = cell(Ng, 1);
gp0           = cell(Ng, 1); % initial
gp1           = cell(Ng, 1); % left
gp2           = cell(Ng, 1); % right

for g = 1:Ng

	% -----------------------------------------
	% 	Check initial conditions
	% -----------------------------------------

	% under world coordinate, calculate the grasp pos for object in q0, qf
	gp1o_w = grasps.points(:, grasp_ids(g), 1);
	gp2o_w = grasps.points(:, grasp_ids(g), 2);

	% Collision between gripper and table
	gp10  = quatOnVec(gp1o_w, q0);
	gp20  = quatOnVec(gp2o_w, q0);
	qfr0  = getProperGraspSimple(gp10, gp20);
	gp1f  = quatOnVec(gp1o_w, qf);
	gp2f  = quatOnVec(gp2o_w, qf);
	qfrf  = getProperGraspSimple(gp1f, gp2f);
	gp0_x = quatOnVec([1 0 0]', qfr0);
	gp0_z = quatOnVec([0 0 1]', qfr0);
	gpf_x = quatOnVec([1 0 0]', qfrf);
	gpf_z = quatOnVec([0 0 1]', qfrf);
	gp10_bottom = gp10 + gp0_x*para.FINGER_OPEN_SPACE - gp0_z*para.FINGER_RADIUS;
	gp20_bottom = gp20 - gp0_x*para.FINGER_OPEN_SPACE - gp0_z*para.FINGER_RADIUS;
	gp1f_bottom = gp1f + gpf_x*para.FINGER_OPEN_SPACE - gpf_z*para.FINGER_RADIUS;
	gp2f_bottom = gp2f - gpf_x*para.FINGER_OPEN_SPACE - gpf_z*para.FINGER_RADIUS;

	if any([gp10_bottom(3)-tablez0 gp20_bottom(3)-tablez0 gp1f_bottom(3)-tablezf gp2f_bottom(3)-tablezf]< para.GRIPPER_Z_LIMIT)
		grasps_flag(g) = -1;
		continue;
	end

	%  Collision between gripper and object
	refz_o = quatOnVec([0 0 1]', grasps.ref_frame(:, grasp_ids(g)));
	refx_o = quatOnVec([1 0 0]', grasps.ref_frame(:, grasp_ids(g)));
	% Find out the slice of collision-free range
	% get seed
	% seed must be within [-pi, pi] around current z
	[~, z0_id]  = getIDinCfRange(qfr0, q0, refz_o, refx_o);
	z0_id_range = (z0_id-90):(z0_id+90);
	cf_range0   = circQuery(grasps.range(:, grasp_ids(g)), z0_id_range)';
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
		collision_free_range(:, i) = singleOutSFRange(grasps.range(:, grasp_ids(g)), seeds(i));
		assert(circQuery(collision_free_range(:, i), seeds(i)) == 1);
	end

	% Check if both initial/final grasp exists 
	% in the same collision free range
	cf_feasible{g}  = true(1, Ncf); % choice of collision free range

	if ~isempty(qg0)
		[~, zg0_id] = getIDinCfRange(qg0, q0, refz_o, refx_o);
		cf_feasible{g} = collision_free_range(zg0_id, :)';
	end

	if ~isempty(qgf)
		[~, zgf_id] = getIDinCfRange(qgf, qf, refz_o, refx_o);
		cf_feasible{g} = (cf_feasible{g}&collision_free_range(zgf_id, :))';
	end

	if ~any(cf_feasible{g})
		grasps_flag = -2;
		continue;
	end

	% -----------------------------------------
	% 	Evaluate the trajectory
	% -----------------------------------------
	points_fr_old = points;
	gp_fr_old     = zeros(3, 1);

	rtype{g}         = zeros(1, Nf); % 1: pivotable, 0: roll
	obj_rotation{g}  = zeros(1, Nf); % rotation of object measured in grasp frame
	tilt_range{g}    = zeros(2, Nf); % min (right), max (left)
	cf_range{g}      = zeros(2, Nf, Ncf); % min (right), max (left)
	grasp_qframes{g} = zeros(4, Nf); % grasp frame at each time step
	stuck{g}         = false(1, Nf);
	gpz{g}           = zeros(1, Nf); % z pos of gripper
	gpxy_delta{g}    = zeros(2, Nf); % xy offset of gripper
	gp1{g}           = zeros(3, Ng); % grasp position (includes offset)
	gp2{g}           = zeros(3, Ng);


	is_feasible = true;
	for fr = 1:Nf
		qp   = qobj(:, fr);
		m_fr = quat2m(qp);

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
		gp1{g}(:, fr)     = gp1_fr;
		gp2{g}(:, fr)     = gp2_fr;

		% 
	    % check z limit
	    % 
		if (gp1_fr(3) < para.GRIPPER_Z_LIMIT + ps_err)||(gp2_fr(3) < para.GRIPPER_Z_LIMIT + ps_err)
			grasps_flag(g) = -4;
			is_feasible = false;
			break;
		end

		% 
		% check tilt angle limit
		% 
		gripper_cone_width = getTiltedGripperCone(gp1_fr, gp2_fr, para.GRIPPER_TILT_LIMIT);
		if isempty(gripper_cone_width)
		    grasps_flag(g) = -3;
			is_feasible = false;
			break;
		end
		tilt_range{g}(1, fr) = - gripper_cone_width;
		tilt_range{g}(2, fr) =   gripper_cone_width;

		% 
		% check collision free range
		% 
		qfr           = getProperGraspSimple(gp1_fr, gp2_fr);
		[z_ang, z_id] = getIDinCfRange(qfr, qp, refz_o, refx_o);
		z_id_range    = (z_id-90):(z_id+90);

		obj_rotation{g}(fr)  = z_ang;
		grasp_qframes{g}(:, fr) = qfr;

		for cf = 1:Ncf
			% check with each range
			if ~cf_feasible{g}(cf)
				continue;
			end
			cf_range_fr = circQuery(collision_free_range(:, cf), z_id_range)';
			start1      = strfind([0,cf_range_fr==1],[0 1]);
			end1        = strfind([cf_range_fr==1,0],[1 0]);
			if isempty(start1)
				cf_feasible{g}(cf) = false;
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
			cf_range{g}(1, fr, cf) = temp_id_range(start1)*pi/180;
			cf_range{g}(2, fr, cf) = temp_id_range(end1)*pi/180;
		end

		if ~any(cf_feasible{g})
			grasps_flag(g) = -6;
			is_feasible = false;
			break;
		end
		% 
		% check pivotablity (Robustly)
		% 
		if strcmp(method, 'pickplace')
			rtype{g}(fr) = 0;
		else
			% measure in grasp frame
			qfr_inv = quatInv(qfr);
			gp1_GF  = quatOnVec(gp1_fr, qfr_inv);
			gp2_GF  = quatOnVec(gp2_fr, qfr_inv);
			assert(abs(gp1_GF(2)-gp2_GF(2))<1e-5);
			com_GF = quatOnVec(com_fr, qfr_inv);

			cpid  = points_fr(3, :) < 2*ps_err + 1e-4;
			cp_fr = points_fr(:, cpid); % potential contact points
			cp_GF = quatOnVec(cp_fr, qfr_inv);

			test_points = [min(cp_GF(2, :)), max(cp_GF(2, :)), com_GF(2)+para.COM_ERR, com_GF(2)-para.COM_ERR];
			test_points_all = [test_points - gp1_GF(2) - para.GP_ERR, test_points - gp1_GF(2) + para.GP_ERR];

			if (~any(test_points_all > 0)) || (~any(test_points_all < 0))
				rtype{g}(fr) = 1;
			end
		end

		% 
		% Check if getting stuck 
		% 	(going down and within friction cone)
		% 
		gp_fr   = (gp1_fr + gp2_fr)/2;
		gpz{g}(fr) = gp_fr(3);
		
		if fr > 1
		    % if gpz(fr) < gpz(fr-1)
		        % going down
				[~, cpid] = min(points_fr(3, :)); % < 2*ps_err + 1e-4;
				cp        = points_fr(:, cpid);
				gp_cp     = gp_fr - cp;

				cones     = atan2(norm(gp_cp(1:2)), abs(gp_cp(3)));
		        if cones < CONE
					stuck{g}(fr)           = true;
					cp_old                 = points_fr_old(:, cpid);
					gp_cp_old              = gp_fr_old - cp_old;
					gpxy_delta{g}(:, fr-1) = gp_cp(1:2) - gp_cp_old(1:2);
		        end
			% end
		else
			gp0{g} = gp_fr;
		end

		points_fr_old = points_fr;	
		gp_fr_old     = gp_fr;
	end

	if ~is_feasible
		continue;
	end
end

% print out summary
dispC(['  --- [Object] # ' num2str(sum(grasps_flag== 1)) ': successful']);
dispC(['  --- --- # ' num2str(sum(grasps_flag==-1)) ': [Pre-checking] initial/final gripper collides with table']);
dispC(['  --- --- # ' num2str(sum(grasps_flag==-2)) ': [Pre-checking] initial/final grasp infeasible in cf_range']);
dispC(['  --- --- # ' num2str(sum(grasps_flag==-3)) ': [q obj optimization] violates gripper tilt angle limit ']);
dispC(['  --- --- # ' num2str(sum(grasps_flag==-4)) ': [q obj optimization] violates gripper Z limit ']);
dispC(['  --- --- # ' num2str(sum(grasps_flag==-5)) ': [q obj optimization] infeasible ']);
dispC(['  --- --- # ' num2str(sum(grasps_flag==-6)) ': [q obj checking] no collision free path ']);

% -----------------------------------------
% 	Compare and pick one grasp
% -----------------------------------------
feasible_id = find(grasps_flag > 0);
Nsol        = length(feasible_id);
if isempty(feasible_id)
	obj_plan = [];
	id_sel   = [];
	return;
end

grasps_score = zeros(Nsol, 1);
for i = 1:Nsol
	minz1 = min(gp1{feasible_id(i)}(3,:));
	minz2 = min(gp2{feasible_id(i)}(3,:));
    grasps_score(i) = min(minz1, minz2) + 0.2*min(tilt_range{feasible_id(i)}(2,:));
end

% dispC('scores: ');
% dispC(grasps_score);

[~, id_sel] = max(grasps_score);
id_sel_feasible = feasible_id(id_sel);

obj_plan.Nf   = Nf;
obj_plan.qobj = qobj;
obj_plan.grasp_qframes = grasp_qframes{id_sel_feasible};
obj_plan.cf_range      = cf_range{id_sel_feasible};     
obj_plan.cf_feasible   = cf_feasible{id_sel_feasible};        
obj_plan.tilt_range    = tilt_range{id_sel_feasible};   
obj_plan.rtype         = rtype{id_sel_feasible};        
obj_plan.obj_rotation  = obj_rotation{id_sel_feasible}; 
obj_plan.stuck         = stuck{id_sel_feasible};        
obj_plan.gpz           = gpz{id_sel_feasible};          
obj_plan.gp0           = gp0{id_sel_feasible};          
obj_plan.gpxy_delta    = gpxy_delta{id_sel_feasible};   

id_sel = grasp_ids(id_sel_feasible);

% % -----------------------------------------
% % 	Optimize the trajectory
% % -----------------------------------------
% N = para.opt_obj_N;

% % init x
% qobj = quatSlerp(q0, qf, (1:N)/N);
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





