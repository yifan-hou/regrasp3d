% inputs:
% 	g: 3x1, direction of gravity projection
% 	goal: 3x1, direction of goal gripper angle
% 	com: 3x1, position of COM
% 	ps: 3xN, points on mesh model (potential contact points with ground)
%   gp: 3x2, 3D position of grasp points in grasp frame
% 	Rw_2d: rotation matrix from world to grasp frame
% 	cmat: NxN, connectivity matrix of points
function	[plan, init_roll_ang] = plan2DObject(g, goal, com, ps, gp, Rw_2d, cmat, para)
% rotate so that gy points down
R      = matBTVec([g(1:2); 0], [0 -1 0]');
g      = R*g; % should be [0 -1 ?]
ps     = R*ps;
gp     = R*gp;
com    = R*com;
goal   = R*goal;  % goal(3) should = 0
init   = R*[0; 1; 0];

gripper_cone_right = [sin(para.GRIPPER_TILT_LIMIT),  cos(para.GRIPPER_TILT_LIMIT), 0]';
gripper_cone_left  = [-sin(para.GRIPPER_TILT_LIMIT), cos(para.GRIPPER_TILT_LIMIT), 0]';

% find contact point
[~, cp_id] = max(g'*ps); % id of contact point in points list
cp         = ps(:,cp_id);

% check initial grasp pos
if init(2) < cos(para.GRIPPER_TILT_LIMIT)
	plan = [];
    init_roll_ang = [];
	return;
end
if ((cp-gp(:,1))'*g < para.GRIPPER_Z_LIMIT) || ((cp-gp(:,2))'*g < para.GRIPPER_Z_LIMIT)
	plan = [];
    init_roll_ang = [];
	return;
end

% Plan
%	dir: direction of object rotation in grasp frame
%	motion: 1xN angle of object to traverse
%   rtype: 1xN, 1: pivotable 0: not pivotable -1: infeasible


% % check if goal is already reachable
% % todo: consider the situation when the initial pose is stable
% if checkPivotability(com, cp, gp, g, para)
% 	if angBTVec([0 1 0]', goal) < para.GRIPPER_TILT_LIMIT
% 		motion = 0;
% 		rtype = 1;
% 		plan.motion = motion;
% 		plan.rtype = rtype;
% 		return;
% 	end
% end

% sample a few angles where goal is overlapping with cone.
z = [0 0 1]'; % when dir = positive, -z is the rotation axis of object

% check and find a feasible path
% dir: direction of object rolling. (assume no sliding)
% try close direction first
if goal(1) < 0
    s_dir  = [1 0 0]';
else
    s_dir  = [-1 0 0]';
end

object_motion_feasible = false;
feasible_dir   = 0;

for d = 1:2
    if object_motion_feasible
        break;
    end
    
    if para.show2Dproblem
        figure(para.show2Dproblem_id); clf; hold on;
        h_gravity   = plot(com(1)+[0 g(1)], com(2)+[0 g(2)], '-r', 'linewidth', 2);
        h_com       = plot(com(1), com(2), '.r', 'markersize', 30);
        h_cp        = plot(cp(1), cp(2), '.k', 'markersize', 35);
        h_ps        = plot(ps(1,:), ps(2,:), '.k', 'markersize', 20);
        h_goal      = plot([0 goal(1)], [0 goal(2)], '-g*', 'markersize',1, 'linewidth', 2);
        h_fingertip = plot(0, 0, '.g', 'markersize',30);
        
        plot([0 init(1)], [0 init(2)], '-b*', 'markersize',1, 'linewidth', 2);
        plot([0 gripper_cone_right(1)], [0 gripper_cone_right(2)], '-.b', 'markersize',1, 'linewidth', 1);
        plot([0 gripper_cone_left(1)], [0 gripper_cone_left(2)], '-.b', 'markersize',1, 'linewidth', 1);
        axis equal
        drawnow
    end
    
    
    % calculate goal angle
    ang2goal = angBTVec([0 1 0]',  goal, z, 1);
    if s_dir(1) < 0
        ang2goal = 2*pi - ang2goal;
    end
%     disp(['Angle to goal:' num2str(ang2goal*180/pi)]);

    % calculate possible initial rolling angle
    if s_dir(1) < 0
        init_roll_ang = angBTVec(init, gripper_cone_left, z, 1);
    else
        init_roll_ang = angBTVec(init, gripper_cone_right, -z, 1);
    end
    
    % initialization
    s_cp    = cp; % current id
    s_cpid  = cp_id;
    s_ps    = ps;
    s_com   = com;
    s_goal  = goal;
    s_angle = 0;
    motion  = [];
    rtype   = [];
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
        
        ang2goal_remaining = ang2goal - s_angle;
        
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
            disp('Rotation failed: constraints violated.');
            break; % stop current rolling, try the other direction
        end
        motion          = [motion s_motion];
        rtype           = [rtype s_rtype];
        
        % yes we can
        % so do the rotation
        R_i     = aa2mat(ang2next, -z*s_dir(1));
        s_com   = R_i*s_com;
        s_goal  = R_i*s_goal;
        s_ps    = R_i*s_ps;
        s_angle = s_angle + ang2next;
        s_cp    = s_ps(:, s_cpid); % old contact points
        
        if para.show2Dproblem
            figure(para.show2Dproblem_id); hold on;
            h_gravity.XData = s_com(1) + [0 g(1)];
            h_gravity.YData = s_com(2) + [0 g(2)];
            h_com.XData     = s_com(1);
            h_com.YData     = s_com(2);
            h_cp.XData      = s_cp(1,:); 
            h_cp.YData      = s_cp(2,:);
            h_ps.XData      = s_ps(1,:);
            h_ps.YData      = s_ps(2,:);
            h_goal.XData    = [0 s_goal(1)];
            h_goal.YData    = [0 s_goal(2)];
            if s_rtype(end) == 1
                h_fingertip.Color = [0 1 0];
            else
                h_fingertip.Color = [0.5 0.5 0];
            end
            axis equal
            drawnow
            disp('Rotated to next contact.');
        end
        
        s_cpid  = adj_p_id(id); % new contact point
        s_cp    = s_ps(:, s_cpid);
        
        
        if flag_finish_now
            % further check the whole trajectory, determine feasibility
            if checkTraj(motion, rtype, init_roll_ang, para)
                % good! record
                feasible_dir   = s_dir(1);
                object_motion_feasible = true;
                disp('2D planning is successful.');
                break;
            else
                s_dir = - s_dir;
                disp('2D planning failed: Required rolling is too large.');
                break;
            end            
        end
        
    end % end rolling
end % end both directions

if ~object_motion_feasible
	plan = [];
    init_roll_ang = [];
	return;
else
	plan.motion = motion;
	plan.rtype  = rtype;
	plan.dir    = feasible_dir;
end

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
    if isempty(change)
        motion = ang;
        rtype = pivotable(1);
    else
        motion(1) = change(1)*para.PIVOTABLE_CHECK_GRANULARITY;
        rtype(1) = pivotable(change(1));
        for i = 2:length(change)
            motion(i) = (change(i) - change(i-1))*para.PIVOTABLE_CHECK_GRANULARITY;
            rtype(i) = pivotable(change(i));
        end        
        motion(end) = ang - change(end)*para.PIVOTABLE_CHECK_GRANULARITY;
        rtype(end) = pivotable(end);
    end
end


function samples = sampleFixStep(a, b, step)
	n = floor((b - a)/step);
	samples = a + (0:n)*step;
end


function feasible = checkTraj(motion, rtype, init_roll_ang, para)
if rtype(end) ~= 1
    feasible = false;
    return;
end
N = length(motion);
first_roll = true;
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
		if first_roll
			first_roll = false;
			if roll_angle > init_roll_ang
				feasible = false;
				return;
			end
		elseif roll_angle > 2*para.GRIPPER_TILT_LIMIT
			feasible = false;
			return;
		end
	end

end
feasible = true;

end
