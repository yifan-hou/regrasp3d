% Select a grasp from @p grasp_ids, and plan the object motion.
% Inputs:
% 	grasp_ids: id of available grasps
% 	q_WO_initial, q_WO_final: initial/final object pose
% 	q_WG_initial, q_WG_final: initial/final grasp pose (could be empty)
%   method_name: 'pickplace' or 'pivoting'
function [obj_plan, id_sel] = planObject(grasp_ids, q_WO_initial, ...
		q_WO_final, q_WG_initial, q_WG_final, method_name)

global mesh grasps pgraph para
obj_plan = [];

kNumOfGrasps = length(grasp_ids);
% symbols
% 	n: number, scalar
% 	ns: 1xn array of numbers
% 	vk: kx1 vector
% 	p: 3x1 vector
% 	ps: 3xn matrix
% 	q: 4x1 vector
% 	bool: bool


ps_OVertices           = pgraph.vertices; % decimated convex hull
p_OCOM                 = mesh.COM;
kPtsErrorBound         = pgraph.err_bound;
kFrictionConeHalfWidth = atan(para.MU);
ps_OVertices_initial   = quatOnVec(ps_OVertices, q_WO_initial);
ps_OVertices_final     = quatOnVec(ps_OVertices, q_WO_final);
n_table_z_initial      = min(ps_OVertices_initial(3,:));
n_table_z_final        = min(ps_OVertices_final(3,:));

% flag_grasps:
% 	  1: successful
% 	  -1: [Pre-checking] initial/final gripper collides with table;
%     -2: [Pre-checking] initial/final grasp infeasible in ns_collision_free_range
%     -3: [q obj optimization] violates gripper tilt angle limit
%     -4: [q obj optimization] violates gripper Z limit
%     -5: [q obj optimization] infeasible
%     -6: [q obj checking] no collision-free path
flags_for_grasps  = ones(kNumOfGrasps, 1);

kNumOfFrames   = 50;
qs_WO = quatSlerp(q_WO_initial, q_WO_final, (1:kNumOfFrames)/kNumOfFrames);

bs_collision_free_gripper_angles = cell(kNumOfGrasps, 1);
flags_mode_choice                = cell(kNumOfGrasps, 1);
ns_obj_rotation_angle            = cell(kNumOfGrasps, 1);
ns_gripper_tilt_ranges           = cell(kNumOfGrasps, 1);
ns_collision_free_range          = cell(kNumOfGrasps, 1);
qs_WG                            = cell(kNumOfGrasps, 1);
bool_is_stuck                    = cell(kNumOfGrasps, 1);
n_WG_z                           = cell(kNumOfGrasps, 1);
v2_WG_xy_delta                   = cell(kNumOfGrasps, 1);
p_WG_initial                     = cell(kNumOfGrasps, 1); % initial
ps_WG_left                       = cell(kNumOfGrasps, 1); % left
ps_WG_right                      = cell(kNumOfGrasps, 1); % right
p_OG_left                        = cell(kNumOfGrasps, 1); % for animation
p_OG_right                       = cell(kNumOfGrasps, 1); %

for g = 1:kNumOfGrasps

	% -----------------------------------------
	% 	Check initial conditions
	% -----------------------------------------
	p_OG_left{g} = grasps.points(:, grasp_ids(g), 1);
	p_OG_right{g} = grasps.points(:, grasp_ids(g), 2);

	% Collision between gripper and table
	p_WG_left_initial      = quatOnVec(p_OG_left{g}, q_WO_initial);
	p_WG_right_initial     = quatOnVec(p_OG_right{g}, q_WO_initial);
	p_WG_left_final        = quatOnVec(p_OG_left{g}, q_WO_final);
	p_WG_right_final       = quatOnVec(p_OG_right{g}, q_WO_final);
	q_WG_initial_proper    = getProperGraspSimple(p_WG_left_initial, p_WG_right_initial);
	q_WG_final_proper      = getProperGraspSimple(p_WG_left_final, p_WG_right_final);
	v3_WG_initial_proper_x = quatOnVec([1 0 0]', q_WG_initial_proper);
	v3_WG_initial_proper_z = quatOnVec([0 0 1]', q_WG_initial_proper);
	v3_WG_final_proper_x   = quatOnVec([1 0 0]', q_WG_final_proper);
	v3_WG_final_proper_z   = quatOnVec([0 0 1]', q_WG_final_proper);
	gp10_bottom = p_WG_left_initial + v3_WG_initial_proper_x*para.FINGER_OPEN_SPACE_0 - v3_WG_initial_proper_z*para.FINGER_RADIUS;
	gp20_bottom = p_WG_right_initial - v3_WG_initial_proper_x*para.FINGER_OPEN_SPACE_0 - v3_WG_initial_proper_z*para.FINGER_RADIUS;
	gp1f_bottom = p_WG_left_final + v3_WG_final_proper_x*para.FINGER_OPEN_SPACE_f - v3_WG_final_proper_z*para.FINGER_RADIUS;
	gp2f_bottom = p_WG_right_final - v3_WG_final_proper_x*para.FINGER_OPEN_SPACE_f - v3_WG_final_proper_z*para.FINGER_RADIUS;

	g0_bottom = min([gp10_bottom(3)-n_table_z_initial  gp20_bottom(3)-n_table_z_initial]);
	gf_bottom = min([gp1f_bottom(3)-n_table_z_final  gp2f_bottom(3)-n_table_z_final]);

    g_bottom = min(g0_bottom, gf_bottom);
	if g_bottom < para.GRIPPER_Z_LIMIT
		if abs(g_bottom - para.GRIPPER_Z_LIMIT) > 0.3*para.FINGER_RADIUS
			flags_for_grasps(g) = -1;
			continue;
		end

		% hack: offset grasp position by a little bit
		gripper_z_shiftup = 0.3*para.FINGER_RADIUS;

		if g0_bottom < gf_bottom
			% grasp 0 needs shift up
			p_WG_left_initial(3)   = p_WG_left_initial(3) + gripper_z_shiftup;
			p_WG_right_initial(3)   = p_WG_right_initial(3) + gripper_z_shiftup;
			p_OG_left{g} = quatOnVec(p_WG_left_initial, quatInv(q_WO_initial));
			p_OG_right{g} = quatOnVec(p_WG_right_initial, quatInv(q_WO_initial));
		else
			p_WG_left_final(3)   = p_WG_left_final(3) + gripper_z_shiftup;
			p_WG_right_final(3)   = p_WG_right_final(3) + gripper_z_shiftup;
			p_OG_left{g} = quatOnVec(p_WG_left_final, quatInv(q_WO_final));
			p_OG_right{g} = quatOnVec(p_WG_right_final, quatInv(q_WO_final));
		end
		% check new grasp
		gp10_bottom = p_WG_left_initial + v3_WG_initial_proper_x*para.FINGER_OPEN_SPACE_0 - v3_WG_initial_proper_z*para.FINGER_RADIUS;
		gp20_bottom = p_WG_right_initial - v3_WG_initial_proper_x*para.FINGER_OPEN_SPACE_0 - v3_WG_initial_proper_z*para.FINGER_RADIUS;
		gp1f_bottom = p_WG_left_final + v3_WG_final_proper_x*para.FINGER_OPEN_SPACE_f - v3_WG_final_proper_z*para.FINGER_RADIUS;
		gp2f_bottom = p_WG_right_final - v3_WG_final_proper_x*para.FINGER_OPEN_SPACE_f - v3_WG_final_proper_z*para.FINGER_RADIUS;
		g0_bottom   = min([gp10_bottom(3)-n_table_z_initial  gp20_bottom(3)-n_table_z_initial]);
		gf_bottom   = min([gp1f_bottom(3)-n_table_z_final  gp2f_bottom(3)-n_table_z_final]);
		g_bottom    = min(g0_bottom, gf_bottom);
	    if g_bottom < para.GRIPPER_Z_LIMIT
	    	% If the hack still doesn't work
            flags_for_grasps(g) = -1;
	    	continue;
	    end
	end

	%  Collision between gripper and object
	v3_OGRef_z = quatOnVec([0 0 1]', grasps.ref_frame(:, grasp_ids(g)));
	v3_OGRef_x = quatOnVec([1 0 0]', grasps.ref_frame(:, grasp_ids(g)));

	% Find out the slice of collision-free range
	% get seed
	% seed must be within [-pi, pi] around current z
	[~, z0_id]  = getIDinCfRange(q_WG_initial_proper, q_WO_initial, v3_OGRef_z, v3_OGRef_x);
	z0_id_range = (z0_id-90):(z0_id+90);
	bs_collision_free_angles_initial   = circQuery(grasps.range(:, grasp_ids(g)), z0_id_range)';
	start1      = strfind([0,bs_collision_free_angles_initial==1],[0 1]);
	end1        = strfind([bs_collision_free_angles_initial==1,0],[1 0]);
	seeds       = round( (end1 + start1)/2);
	seeds       = z0_id_range(seeds);
	kNumOfCollisionFreeRegions         = length(seeds);

	collision_free_range = zeros(360, kNumOfCollisionFreeRegions);
	for i = 1:kNumOfCollisionFreeRegions
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
	bs_collision_free_gripper_angles{g}  = true(1, kNumOfCollisionFreeRegions); % choice of collision free range

	if ~isempty(q_WG_initial)
		[~, zg0_id] = getIDinCfRange(q_WG_initial, q_WO_initial, v3_OGRef_z, v3_OGRef_x);
		bs_collision_free_gripper_angles{g} = collision_free_range(zg0_id, :)';
	end

	if ~isempty(q_WG_final)
		[~, zgf_id] = getIDinCfRange(q_WG_final, q_WO_final, v3_OGRef_z, v3_OGRef_x);
		bs_collision_free_gripper_angles{g} = (bs_collision_free_gripper_angles{g}&collision_free_range(zgf_id, :))';
	end

	if ~any(bs_collision_free_gripper_angles{g})
		flags_for_grasps = -2;
		continue;
	end

	% -----------------------------------------
	% 	Evaluate the trajectory
	% -----------------------------------------
	ps_WVertices_fr_old = ps_OVertices;
	p_WG_fr_old           = zeros(3, 1);

	flags_mode_choice{g}       = zeros(1, kNumOfFrames); % 1: pivotable, 0: roll
	ns_obj_rotation_angle{g}   = zeros(1, kNumOfFrames); % rotation of object measured in grasp frame
	ns_gripper_tilt_ranges{g}  = zeros(2, kNumOfFrames); % min (right), max (left)
	ns_collision_free_range{g} = zeros(2, kNumOfFrames, kNumOfCollisionFreeRegions); % min (right), max (left)
	qs_WG{g}                   = zeros(4, kNumOfFrames); % grasp frame at each time step
	bool_is_stuck{g}           = false(1, kNumOfFrames);
	n_WG_z{g}                  = zeros(1, kNumOfFrames); % z pos of gripper
	v2_WG_xy_delta{g}          = zeros(2, kNumOfFrames); % xy offset of gripper
	ps_WG_left{g}              = zeros(3, kNumOfFrames); % grasp position (includes offset)
	ps_WG_right{g}             = zeros(3, kNumOfFrames);

	b_grasp_is_feasible = true;
	for fr = 1:kNumOfFrames
		q_WO_fr   = qs_WO(:, fr);
		m_WO_fr = quat2m(q_WO_fr);

		% rotate and compute:
		% 	points on the object
		%	grasp point
		% 	COM
		% 	contact point on the table
		ps_WVertices_fr = m_WO_fr*ps_OVertices;
		p_WCOM_fr       = m_WO_fr*p_OCOM;
		p_WG_left_fr    = m_WO_fr*p_OG_left{g};
		p_WG_right_fr   = m_WO_fr*p_OG_right{g};

		% offset z table = 0
		zoffset			     = min(ps_WVertices_fr(3,:));
		ps_WVertices_fr(3,:) = ps_WVertices_fr(3,:) - zoffset;
		p_WG_left_fr(3)      = p_WG_left_fr(3) - zoffset;
		p_WG_right_fr(3)     = p_WG_right_fr(3) - zoffset;
		p_WCOM_fr(3)         = p_WCOM_fr(3) - zoffset;

		ps_WG_left{g}(:, fr)  = p_WG_left_fr;
		ps_WG_right{g}(:, fr) = p_WG_right_fr;

		%
	    % check z limit
	    %
		if (p_WG_left_fr(3) < para.GRIPPER_Z_LIMIT + kPtsErrorBound)||(p_WG_right_fr(3) < para.GRIPPER_Z_LIMIT + kPtsErrorBound)
			flags_for_grasps(g) = -4;
			b_grasp_is_feasible = false;
			break;
		end

		%
		% check tilt angle limit
		%
		gripper_cone_width = getTiltedGripperCone(p_WG_left_fr, p_WG_right_fr, para.GRIPPER_TILT_LIMIT);
		if isempty(gripper_cone_width)
		    flags_for_grasps(g) = -3;
			b_grasp_is_feasible = false;
			break;
		end
		ns_gripper_tilt_ranges{g}(1, fr) = - gripper_cone_width;
		ns_gripper_tilt_ranges{g}(2, fr) =   gripper_cone_width;

		%
		% check collision free range
		%
		q_WG_fr       = getProperGraspSimple(p_WG_left_fr, p_WG_right_fr);
		[z_ang, z_id] = getIDinCfRange(q_WG_fr, q_WO_fr, v3_OGRef_z, v3_OGRef_x);
		z_id_range    = (z_id-90):(z_id+90);

		ns_obj_rotation_angle{g}(fr)  = z_ang;
		qs_WG{g}(:, fr) = q_WG_fr;

		for cf = 1:kNumOfCollisionFreeRegions
			% check with each range
			if ~bs_collision_free_gripper_angles{g}(cf)
				continue;
			end
			bs_collision_free_angle_now = circQuery(collision_free_range(:, cf), z_id_range)';
			start1      = strfind([0,bs_collision_free_angle_now==1],[0 1]);
			end1        = strfind([bs_collision_free_angle_now==1,0],[1 0]);
			if isempty(start1)
				bs_collision_free_gripper_angles{g}(cf) = false;
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
			ns_collision_free_range{g}(1, fr, cf) = temp_id_range(start1)*pi/180;
			ns_collision_free_range{g}(2, fr, cf) = temp_id_range(end1)*pi/180;
		end

		if ~any(bs_collision_free_gripper_angles{g})
			flags_for_grasps(g) = -6;
			b_grasp_is_feasible = false;
			break;
		end

		%
		% check pivotablity (Robustly)
		%
		if strcmp(method_name, 'pickplace')
			flags_mode_choice{g}(fr) = 0;
		else
			% measure in grasp frame
			q_GW_fr       = quatInv(q_WG_fr);
			p_GG_left_fr  = quatOnVec(p_WG_left_fr, q_GW_fr);
			p_GG_right_fr = quatOnVec(p_WG_right_fr, q_GW_fr);
			assert(abs(p_GG_left_fr(2)-p_GG_right_fr(2))<1e-5);
			p_GCOM_fr = quatOnVec(p_WCOM_fr, q_GW_fr);

			% potential contact points
			cpid  = ps_WVertices_fr(3, :) < 2*kPtsErrorBound + 1e-4;
			ps_GContacts = quatOnVec(ps_WVertices_fr(:, cpid), q_GW_fr);

			test_points = [min(ps_GContacts(2, :)), max(ps_GContacts(2, :)), p_GCOM_fr(2)+para.COM_ERR, p_GCOM_fr(2)-para.COM_ERR];
			test_points_all = [test_points - p_GG_left_fr(2) - para.GP_ERR, test_points - p_GG_left_fr(2) + para.GP_ERR];

			if (~any(test_points_all > 0)) || (~any(test_points_all < 0))
				flags_mode_choice{g}(fr) = 1;
			end
		end

		%
		% Check if getting stuck
		% 	(going down and within friction cone)
		%
		p_WG_fr      = (p_WG_left_fr + p_WG_right_fr)/2;
		n_WG_z{g}(fr) = p_WG_fr(3);

		if fr > 1
            if (flags_mode_choice{g}(fr)==1) && (n_WG_z{g}(fr) < n_WG_z{g}(fr-1))
                % Pivoting, going down
                [~, cpid] = min(ps_WVertices_fr(3, :)); % < 2*kPtsErrorBound + 1e-4;
                cp        = ps_WVertices_fr(:, cpid);
                gp_cp     = p_WG_fr - cp;

                cones     = atan2(norm(gp_cp(1:2)), abs(gp_cp(3)));
                if cones < kFrictionConeHalfWidth
                    bool_is_stuck{g}(fr)           = true;
                    cp_old                 = ps_WVertices_fr_old(:, cpid);
                    gp_cp_old              = p_WG_fr_old - cp_old;
                    v2_WG_xy_delta{g}(:, fr-1) = gp_cp(1:2) - gp_cp_old(1:2);
                end
            end
		else
			p_WG_initial{g} = p_WG_fr;
		end

		ps_WVertices_fr_old = ps_WVertices_fr;
		p_WG_fr_old     = p_WG_fr;
	end

	if ~b_grasp_is_feasible
		continue;
	end
end

% print out summary
dispC(['  --- [Object] # ' num2str(sum(flags_for_grasps== 1)) ': successful']);
dispC(['  --- --- # ' num2str(sum(flags_for_grasps==-1)) ': [Pre-checking] initial/final gripper collides with table']);
dispC(['  --- --- # ' num2str(sum(flags_for_grasps==-2)) ': [Pre-checking] initial/final grasp infeasible in ns_collision_free_range']);
dispC(['  --- --- # ' num2str(sum(flags_for_grasps==-3)) ': [q obj optimization] violates gripper tilt angle limit ']);
dispC(['  --- --- # ' num2str(sum(flags_for_grasps==-4)) ': [q obj optimization] violates gripper Z limit ']);
dispC(['  --- --- # ' num2str(sum(flags_for_grasps==-5)) ': [q obj optimization] infeasible ']);
dispC(['  --- --- # ' num2str(sum(flags_for_grasps==-6)) ': [q obj checking] no collision free path ']);

% -----------------------------------------
% 	Compare and pick one grasp
% -----------------------------------------
feasible_id = find(flags_for_grasps > 0);
Nsol        = length(feasible_id);
if isempty(feasible_id)
	obj_plan = [];
	id_sel   = [];
	return;
end

disp(grasp_ids(feasible_id));

grasps_score = zeros(Nsol, 1);
for i = 1:Nsol
	minz1 = min(ps_WG_left{feasible_id(i)}(3,:));
	minz2 = min(ps_WG_right{feasible_id(i)}(3,:));
%     grasps_score(i) = 0.003*min(minz1, minz2) + 1.0*min(ns_gripper_tilt_ranges{feasible_id(i)}(2,:));
    grasps_score(i) = 1.003*min(minz1, minz2) + 0.0*min(ns_gripper_tilt_ranges{feasible_id(i)}(2,:));
end

[~, id_sel]     = sort(grasps_score, 'descend');
if length(id_sel) > para.N_Grasps_Attempt
	id_sel = id_sel(1:para.N_Grasps_Attempt);
end

id_sel_feasible = feasible_id(id_sel);

obj_plan = cell(length(id_sel),1);
for gps = 1:length(id_sel)
	obj_plan{gps}.kNumOfFrames                     = kNumOfFrames;
	obj_plan{gps}.qs_WO                            = qs_WO;
	obj_plan{gps}.qs_WG                            = qs_WG{id_sel_feasible(gps)};
	obj_plan{gps}.p_WG_initial                     = p_WG_initial{id_sel_feasible(gps)};
	obj_plan{gps}.n_WG_z                           = n_WG_z{id_sel_feasible(gps)};
	obj_plan{gps}.v2_WG_xy_delta                   = v2_WG_xy_delta{id_sel_feasible(gps)};
	obj_plan{gps}.ns_collision_free_range          = ns_collision_free_range{id_sel_feasible(gps)};
	obj_plan{gps}.bs_collision_free_gripper_angles = bs_collision_free_gripper_angles{id_sel_feasible(gps)};
	obj_plan{gps}.ns_gripper_tilt_ranges           = ns_gripper_tilt_ranges{id_sel_feasible(gps)};
	obj_plan{gps}.flag_mode_choice                 = flags_mode_choice{id_sel_feasible(gps)};
	obj_plan{gps}.ns_obj_rotation_angle            = ns_obj_rotation_angle{id_sel_feasible(gps)};
	obj_plan{gps}.bool_is_stuck                    = bool_is_stuck{id_sel_feasible(gps)};
	obj_plan{gps}.p_OG_left                        = p_OG_left{id_sel_feasible(gps)};
	obj_plan{gps}.p_OG_right                       = p_OG_right{id_sel_feasible(gps)};
end

id_sel = grasp_ids(id_sel_feasible);

% % -----------------------------------------
% % 	Optimize the trajectory
% % -----------------------------------------
% N = para.opt_obj_N;

% % init x
% qs_WO = quatSlerp(q_WO_initial, q_WO_final, (1:N)/N);
% x            = (2*rand(3*N,1) - 1)*pi;

% global paraOpt_OBJ
% paraOpt_OBJ.delta_theta   = para.opt_obj_con_delta_theta; % rad
% paraOpt_OBJ.q_WO_initial            = q_WO_initial;
% paraOpt_OBJ.q_WO_final            = q_WO_final;
% paraOpt_OBJ.qf_inv        = quatInv(q_WO_final);
% paraOpt_OBJ.Gp1o          = p_OG_left;
% paraOpt_OBJ.Gp2o          = p_OG_right;
% paraOpt_OBJ.Gp_tilt_limit = para.GRIPPER_TILT_LIMIT;
% paraOpt_OBJ.cost_dq_k     = para.opt_obj_cost_k_dq;
% paraOpt_OBJ.cost_tilt_k   = para.opt_obj_cost_k_tilt;
% paraOpt_OBJ.exactqf_k     = exactq;
% %	exactq: true if we want to attain q_WO_final exactly. false if we don't care yaw angle

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
% qs_WO        = [cos(theta); temp.*n];

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

sf_range = ids;

end


function [z0_ang, z0_id] = getIDinCfRange(q_WG, q_WO, v3_OGRef_z, v3_OGRef_x)
	qgz    = quatOnVec([0 0 1]', q_WG);
	qgz_o  = quatOnVec(qgz, quatInv(q_WO));
	z0_ang = angBTVec(v3_OGRef_z, qgz_o, v3_OGRef_x, 1);
	z0_id  = round(180/pi*z0_ang);
end





