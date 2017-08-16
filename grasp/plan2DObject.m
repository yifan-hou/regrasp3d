% inputs:
% 	g: 3x1, direction of gravity projection
% 	goal: 3x1, direction of goal gripper angle
% 	com: 3x1, position of COM
%   gp: 3x2, 3D position of grasp points in grasp frame
%   Rw_2d: rotation matrix from world to grasp frame
% 	ps: 3xN, points on convex hull (potential contact points with ground)
% 	cmat: NxN, connectivity matrix of points on convex hull
% outputs:
%   flag:
%       1: success
%       -1: required rolling exceeds gripper tilt limit
%       -3: initial grasp pos violates gripper tilt limit 
%       -4: initial grasp pos violates gripper Z limit 
function	[plan, init_gripper_ang, flag] = plan2DObject(g, goal, com, gp, Rw_2d, ps, ps_err, cmat, cf_range, cf_init_id, gripper_cone_width, para)
% rotate so that gy points down
R    = matBTVec([g(1:2); 0], [0 -1 0]');
g    = R*g; % should be [0 -1 ?]
ps   = R*ps;
gp   = R*gp;
com  = R*com;
goal = R*goal;  % goal(3) should = 0
init = R*[0; 1; 0];

% check initial grasp orientation
if norm(g(1:2)) < 1e-2
    plan             = [];
    init_gripper_ang = [];
    flag             = -3;
	return;
end
% check initial grasp orientation
if init(2) < cos(gripper_cone_width)
    plan             = [];
    init_gripper_ang = [];
    flag             = -3;
	return;
end


init_gripper_ang = angBTVec([g(1:2); 0], [0 -1 0]', [0 0 1]');
cf_zero_id       = cf_init_id - round(180/pi*init_gripper_ang);

gripper_cone_right = [sin(gripper_cone_width),  cos(gripper_cone_width), 0]';
gripper_cone_left  = [-sin(gripper_cone_width), cos(gripper_cone_width), 0]';

% find contact point
[~, cp_id] = max(g'*ps); % id of contact point in points list
cp         = ps(:,cp_id);


% check initial grasp height
if ((cp-gp(:,1))'*g < para.GRIPPER_Z_LIMIT) || ((cp-gp(:,2))'*g < para.GRIPPER_Z_LIMIT)
    plan             = [];
    init_gripper_ang = [];
    flag             = -4;
	return;
end


% Plan
%	dir: direction of object rotation in grasp frame
%	motion: 1xN angle of object to traverse
%   rtype: 1xN, 1: pivotable 0: not pivotable -1: infeasible


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
feasible_dir           = 0;

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
    if s_dir(1) > 0
        ang2goal = angBTVec(gripper_cone_right,  goal, z, 1);
    else % s_dir(1) < 0
        ang2goal = angBTVec(gripper_cone_left,  goal, -z, 1);
    end
    % disp(['Angle to goal:' num2str(ang2goal*180/pi)]);

    % calculate possible initial rolling angle
    if s_dir(1) < 0
        init_roll_ang = angBTVec(init, gripper_cone_left, z, 1);
    else
        init_roll_ang = angBTVec(init, gripper_cone_right, -z, 1);
    end
    
    % initialization
    s_cp     = cp; % current id
    s_cpid   = cp_id;
    s_ps     = ps;
    s_com    = com;
    s_goal   = goal;
    s_angle  = 0;
    motion   = [];
    rtype    = [];
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
                adj_p_id     = [find(tempid) find(tempid2)];
                contact_vecs = [s_ps(:, tempid) - s_ps(:, s_cpid), s_ps(:, tempid2) - s_ps(:, cpid2)];
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
            ang2next        = ang2goal_remaining;
            flag_finish_now = 1;
        end
        
        % can we do this rotation?
        [s_motion, s_rtype] = calRotation(s_com, s_cp, ang2next, -z*s_dir(1), gp, g, ps_err, para);
        if s_rtype(1) == -1
            % no we can't
            s_dir = - s_dir;
            flag  = -1; % flag could be re-written later, if success
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
        end
        
        s_cpid  = adj_p_id(id); % new contact point
        s_cp    = s_ps(:, s_cpid);
        
        if flag_finish_now
            % further check the whole trajectory, determine feasibility
            [feasible, motion, rtype, cf_range_at_roll]  = checkTraj(motion, rtype, init_roll_ang, s_dir(1), ...
                                                             cf_range, cf_zero_id, gripper_cone_width);
            if feasible
                % good! record
                feasible_dir   = s_dir(1);
                object_motion_feasible = true;
                flag = 1; % successful
                break;
            else
                s_dir = - s_dir;
                flag = -1; 
                break;
            end            
        end
        
    end % end rolling
end % end both directions

if ~object_motion_feasible
    plan          = [];
    init_gripper_ang = [];
	return;
else
    plan.motion           = motion;
    plan.rtype            = rtype;
    plan.dir              = feasible_dir;
    plan.cf_range_at_roll = cf_range_at_roll;

    % check
    init_goal_ang = angBTVec([0 1 0]', goal, [0 0 1]');
    if feasible_dir > 0
        final_goal_ang = init_goal_ang - sum(motion);
    else
        final_goal_ang = init_goal_ang + sum(motion)
    end
    disp(['final_goal_ang: ' num2str(180/pi*final_goal_ang)]);

end

end

% -1: infeasible
%  0: can do roll
%  1: can do pivot
function pivotable = checkPivotability(com, cp, gp, g, ps_err, para)
	% 0: only roll
    pivotable = 0;
    % 1: can pivot
	if ((cp(1)-ps_err)*com(1) > 0) && ((cp(1)+ps_err)*com(1) > 0)
		pivotable = 1;
	end
    % -1: gripper limit violated
	if ((cp-gp(:,1))'*g < para.GRIPPER_Z_LIMIT)||((cp-gp(:,2))'*g < para.GRIPPER_Z_LIMIT)
		pivotable = -1;
	end

end

% z: rotation axis of object
function [motion, rtype] = calRotation(com, cp, ang, z, gp, g, ps_err, para)
	N = floor(ang/para.PIVOTABLE_CHECK_GRANULARITY);
	pivotable = zeros(1, N+1);
	for i = 0:N
		ang_i = i*para.PIVOTABLE_CHECK_GRANULARITY;
		com_i = aaOnVec(com, ang_i, z);
		cp_i  = aaOnVec(cp, ang_i, z);
		pivotable(i+1) = checkPivotability(com_i, cp_i, gp, g, ps_err, para);
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
        rtype(1)  = pivotable(change(1));
        for i = 2:length(change)
            motion(i) = (change(i) - change(i-1))*para.PIVOTABLE_CHECK_GRANULARITY;
            rtype(i)  = pivotable(change(i));
        end        
        motion(end) = ang - change(end)*para.PIVOTABLE_CHECK_GRANULARITY;
        rtype(end) = pivotable(end);
    end
end


% function samples = sampleFixStep(a, b, step)
% 	n = floor((b - a)/step);
% 	samples = a + (0:n)*step;
% end

% combine adjacent motions with same type
% check roll angles, check collision.
% Input
%   motion: object motion at each stage. Nonnegative
% Output
%   feasible: bool
%   motion_: nx1 concatenated motion stages
%   type_: nx1 types of each stages, no consecutive value. 010101
%   range_: 360xk, collision-free range at the beginning of each roll stage
function [feasible, motion_, type_, range_] = checkTraj(motion, rtype, init_roll_ang, dir, cf_range, cf_zero_id, gripper_cone_width)
N            = length(motion);
first_roll   = true;
feasible     = true;
motion_      = zeros(N, 1);
type_        = zeros(N, 1);
stage_count_ = 0;

roll_count_ = 0;
Nrange      = round(gripper_cone_width*2*180/pi) + 1;
range_      = zeros(Nrange, N);

i = 1;
while true
    accu_angle          = 0;
    type_(stage_count_+1) = rtype(i);
    while true
        accu_angle = accu_angle + motion(i);
        i = i + 1;
        if (i > N) || (rtype(i) ~= type_(stage_count_+1))
            break;
        end
    end

    if type_(stage_count_+1) == 0 % roll
        roll_count_ = roll_count_ + 1;
        % -----------------------------------
        %   check roll angle limit
        % -----------------------------------
        if (first_roll && (accu_angle > init_roll_ang)) || (accu_angle > 2*gripper_cone_width)
            feasible = false;
            break;
        end
        accu_angle_id = round(accu_angle*180/pi);
        % -----------------------------------
        %   check collision
        % -----------------------------------
        zone_start             = cf_zero_id + round(180/pi*(dir*sum(motion_) - gripper_cone_width));
        zone_end               = zone_start + Nrange - 1;
        zone                   = zone_start:zone_end;
        range_(:, roll_count_) = circQuery(cf_range, zone);
        if dir
            range_(1:accu_angle_id, roll_count_) = false; 
        else
            range_(end-accu_angle_id:end, roll_count_) = false; 
        end

        if ~any(range_(:, roll_count_))
            feasible = false;
            break;
        end
    end

    first_roll            = false;
    motion_(stage_count_+1) = accu_angle;
    stage_count_          = stage_count_ + 1;

    if i > N
        break;
    end
end

if ~feasible
    motion_ = [];
    type_   = [];
    range_  = [];
    return;
end

motion_ = motion_(1:stage_count_);
type_   = type_(1:stage_count_);
range_  = range_(:, 1:roll_count_);

end
