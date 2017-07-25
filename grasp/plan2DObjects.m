% inputs:
% 	g: 3x1, direction of gravity projection
% 	goal: 3x1, direction of goal gripper angle
% 	com: 3x1, position of COM
% 	ps: 3xN, points on mesh model (potential contact points with ground)
%   gp: 3x2, 3D position of grasp points in grasp frame
% 	Rw_2d: rotation matrix from world to grasp frame
% 	cmat: NxN, connectivity matrix of points
function	plan = plan2DObject(g, goal, goal_l, goal_r, com, ps, gp, Rw_2d, cmat, para)
% rotate so that gy points down
R      = matBTVec([g(1:2); 0], [0 -1 0]');
g      = R*g; % should be [0 -1 ?]
ps     = R*ps;
gp     = R*gp;
com    = R*com;
goal   = R*goal;  % goal(3) should = 0
goal_l = R*goal_l;
goal_r = R*goal_r;
init   = R*[0; 1; 0];

gripper_cone_right = [sin(para.GRIPPER_TILT_LIMIT),  cos(para.GRIPPER_TILT_LIMIT), 0]';
gripper_cone_left  = [-sin(para.GRIPPER_TILT_LIMIT), cos(para.GRIPPER_TILT_LIMIT), 0]';

% find contact point
[~, cp_id] = max(g'*ps); % id of contact point in points list
cp         = ps(:,cp_id);

if para.show2Dproblem
	figure(para.show2Dproblem_id); clf; hold on;
	plot(com(1)+[0 g(1)], com(2)+[0 g(2)], '-r', 'linewidth', 2);
	plot(com(1), com(2), '.r', 'markersize', 30);
	plot(cp(1), cp(2), '.k', 'markersize', 30);
	plot(ps(1,:), ps(2,:), '.k', 'markersize', 20);
	plot(0, 0, '.g', 'markersize',30);

	plot([0 goal(1)], [0 goal(2)], '-g*', 'markersize',1, 'linewidth', 2);
	plot([0 goal_l(1)], [0 goal_l(2)], '-.g', 'markersize',1, 'linewidth', 1);
	plot([0 goal_r(1)], [0 goal_r(2)], '-.g', 'markersize',1, 'linewidth', 1);
	plot([0 init(1)], [0 init(2)], '-b*', 'markersize',1, 'linewidth', 2);
	plot([0 gripper_cone_right(1)], [0 gripper_cone_right(2)], '-.b', 'markersize',1, 'linewidth', 1);
	plot([0 gripper_cone_left(1)], [0 gripper_cone_left(2)], '-.b', 'markersize',1, 'linewidth', 1);
	axis equal
    drawnow
end

% check initial grasp pos
if init(2) < cos(para.GRIPPER_TILT_LIMIT)
	plan = [];
	return;
end
if ((cp-gp(:,1))'*g < para.GRIPPER_Z_LIMIT) || ((cp-gp(:,2))'*g < para.GRIPPER_Z_LIMIT)
	plan = [];
	return;
end

% Plan
%	dir: direction of object rotation in grasp frame
%	motion: 1xN angle of object to traverse
%   rtype: 1xN, 1: pivotable 0: not pivotable -1: infeasible


% check if goal is already reachable
% todo: consider the situation when the initial pose is stable
if checkPivotability(com, cp, gp, g, para)
	if angBTVec([0 1 0]', goal) < para.GRIPPER_TILT_LIMIT
		motion = 0;
		rtype = 1;
		plan.motion = motion;
		plan.rtype = rtype;
		return;
	end
end

% sample a few angles where goal is overlapping with cone.
z = [0 0 1]'; % when dir = positive, -z is the rotation axis of object
ang2goal_l = angBTVec(gripper_cone_left,  goal_r, z, 1);
ang2goal_r = angBTVec(gripper_cone_right, goal_l, z, 1);
ang2goal   = sampleFixStep(ang2goal_l, ang2goal_r, para.GOALSAMPLEDENSITY2D);

% % check their pose feasibility
% feasible_angle = zeros(1, length(ang2goal));
% for i = 1:length(ang2goal)
% 	% rotate
% 	g_com     = aaOnVec(s_com, ang2goal(i), z);
% 	g_ps      = aaOnVec(s_ps, ang2goal(i), z);
% 	[~, gpid] = min(g_ps(2,:));
% 	g_cp      = g_ps(:,gpid);
% 	% todo: collision checking here
% 	if checkPivotability(g_com, g_cp, para) == 1
% 		feasible_angle(i) = 1;
% 	end
% end
% ang2goal = ang2goal(feasible_angle);

% check and find a feasible path to each feasible angle
% dir: direction of object rolling. (assume no sliding)
% try close direction first
if goal(1) < 0
    s_dir  = [1 0 0]';
else
    s_dir  = [-1 0 0]';
end

motion         = cell(1,  length(ang2goal));
rtype          = cell(1,  length(ang2goal));
feasible_angle = zeros(1, length(ang2goal));
feasible_dir   = zeros(1, length(ang2goal));
for i = 1:length(ang2goal)

    for d = 1:2
        % calculate goal angle
        if s_dir(1) < 0
            ang2goal(i) = 2*pi - ang2goal(i);
        end
        
        % initialization
        s_cp     = cp; % current id
        s_cpid   = cp_id;
        s_ps     = ps;
        s_com    = com;
        s_goal   = goal;
        s_angle  = 0;
        motion_i = [];
        rtype_i  = [];
        % begin rolling
        while true
            % get next contact
            adj_p_id = find(cmat(s_cpid, :));
            
            contact_vecs = s_ps(:, adj_p_id) - s_ps(:, s_cpid);
            if abs(g(3))<1e-5
                % gravity points down
                % could have 2 points overlapping
                id_same = find(normByCol(contact_vecs(1:2,:)) < 1e-5);
                assert(length(id_same)<=1);
                if ~isempty(id_same)
                    cpid2        = adj_p_id(id_same);
                    tempid       = cmat(s_cpid,:); tempid(cpid2) = 0;
                    tempid2      = cmat(cpid2,:); tempid2(s_cpid) = 0;
                    adj_p_id = [find(tempid) find(tempid2)];
                    contact_vecs = [s_ps(:, find(tempid)) - s_ps(:, s_cpid), s_ps(:, find(tempid2)) - s_ps(:, cpid2)];
                end
            end
            
            % we can rotate forward.
            % check how much do we need to rotate...can we reach the goal?
            contact_vecs_angles = zeros(1, size(contact_vecs,2));
            axis_w = -Rw_2d'*[0; 0; 1]*s_dir(1);
            for v = 1:size(contact_vecs,2)
                temp_ang = rot2plane(Rw_2d'*contact_vecs(:,v), axis_w, Rw_2d'*s_dir);
                if isempty(temp_ang)
                    contact_vecs_angles(v) = NaN;
                else
                    contact_vecs_angles(v) = temp_ang(1);
                end
            end
            [ang2next, id] = min(contact_vecs_angles);
            
            % if abs(ang2next) < 1e-5
            %     % There is another contact points in the front. don't rotate at all
            %     % register contact point to this point
            %     % repeat this step will eventually move the contact point to the very front
            %     s_cpid = adj_p_id(id);
            %     s_cp   = s_ps(:, s_cpid);
            %     continue;
            % end
            
            ang2goal_remaining = ang2goal(i) - s_angle;
            
            flag_finish_now = 0;
            if ang2goal_remaining < ang2next
                ang2next = ang2goal_remaining;
                flag_finish_now = 1;
            end
            
            % can we do this rotation?
            [s_motion, s_rtype] = calRotation(s_com, s_cp, ang2next, -z*s_dir(1), gp, g, para);
            if s_rtype(1) == -1
                % no we can't
                s_dir = - s_dir;
                break; % stop current rolling, try the other direction
            end
            motion_i          = [motion_i s_motion];
            rtype_i           = [rtype_i s_rtype];
            
            % yes we can
            if flag_finish_now
                % further check the trajectory, determine feasibility
                if checkTraj(motion_i, rtype_i,para)
                    % good! record
                    motion{i}         = motion_i;
                    rtype{i}          = rtype_i;
                    feasible_dir(i)   = s_dir(1);
                    feasible_angle(i) = true;
                    break;
                else
                    s_dir = - s_dir;
                    break;
                end
            else
                % do the rotation
                R_i     = aa2mat(ang2next, -z*s_dir(1));
                s_com   = R_i*s_com;
                s_goal  = R_i*s_goal;
                s_ps    = R_i*s_ps;
                s_angle = s_angle + ang2next;
                s_cpid  = adj_p_id(id); % next contact point
                s_cp    = s_ps(:, s_cpid);
                if para.show2Dproblem
                    figure(para.show2Dproblem_id); hold on;
                    plot(s_com(1)+[0 g(1)], s_com(2)+[0 g(2)], '-r', 'linewidth', 2);
                    plot(s_com(1), s_com(2), '.r', 'markersize', 30);
                    plot(s_cp(1), s_cp(2), '.k', 'markersize', 30);
                    plot(s_ps(1,:), s_ps(2,:), '.k', 'markersize', 15);
                    plot([0 s_goal(1)], [0 s_goal(2)], '-g*', 'markersize',1, 'linewidth', 2);
                    axis equal
                end
            end
        end % end rolling
    end % end both directions
end % end for all angles

% pick the one closest to vertical
id_feasibles = find(feasible_angle);
[~, id_chosen] = min(abs(id_feasibles - 0.5*length(feasible_angle)));

plan.motion = motion{id_chosen};
plan.rtype  = rtype{id_chosen};
plan.dir    = feasible_dir(id_chosen);

end


function pivotable = checkPivotability(com, cp, gp, g, para)
	pivotable = 0;
	if com(1)*cp(1) > 0
		pivotable = 1;
	end
	if ((cp-gp(:,1))'*g < para.GRIPPER_Z_LIMIT)||((cp-gp(:,2))'*g < para.GRIPPER_Z_LIMIT)
		pivotable = -1;
	end
end

% z: rotation axis of object
function [motion, rtype] = calRotation(com, cp, ang, z, gp, g, para)
	N = floor(ang/para.PIVOTABLE_CHECK_GRANULARITY);
	pivotable = zeros(1, N+1);
	for i = 0:N
		ang_i = i*para.PIVOTABLE_CHECK_GRANULARITY;
		com_i = aaOnVec(com, ang_i, z);
		cp_i  = aaOnVec(cp, ang_i, z);
		pivotable(i+1) = checkPivotability(com_i, cp_i, gp, g, para);
	end

	if any(pivotable < 0)
		% infeasible
		rtype = -1;
		motion = 0;
		return;
	end

	change = find(pivotable(1:end-1) ~= pivotable(2:end));
	motion = zeros(1, length(change)+1);
	rtype  = zeros(1, length(change)+1);
    for i = 1:length(change)
        motion(i) = change(i)*para.PIVOTABLE_CHECK_GRANULARITY;
        rtype(i) = pivotable(change(i));
    end
    if isempty(change)
        motion(end) = ang;
    else
        motion(end) = ang - change(end)*para.PIVOTABLE_CHECK_GRANULARITY;
    end
	rtype(end) = pivotable(end);
end


function samples = sampleFixStep(a, b, step)
	n = floor((b - a)/step);
	samples = a + (0:n)*step;
end


function feasible = checkTraj(motion, rtype, para)
N = length(motion);
for i = 1:N
	roll_angle = 0;
	if rtype(i) == 0
		while true
			roll_angle = roll_angle + motion(i);
			i = i + 1;
			if (i > N) || (rtype(i) ~= 0)
				break;
			end
		end
		if roll_angle > 2*para.GRIPPER_TILT_LIMIT
			feasible = false;
			return;
		end
	end

end
feasible = true;

end
