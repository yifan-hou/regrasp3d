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
%        1: success
%       -1: required rolling exceeds gripper tilt limit
%       -3: initial grasp pos violates gripper tilt limit 
%       -4: initial grasp pos violates gripper Z limit 
%   Plan
%       (needs to be defined) dir: direction of object rotation in grasp frame
%       motion: 1xN angle of object to traverse
%       rtype: 1xN, 1: pivotable 0: not pivotable -1: infeasible
function    [plan, init_roll_ang, flag] = plan2DObject(dir_rot, g, init, goal, com, gp, ps, Rw_2d, ps_err, gripper_cone_width, para)

z = [0 0 1]'; % when dir = positive, -z is the rotation axis of object


CONE = atan(para.MU);

gripper_cone_right = [sin(gripper_cone_width),  cos(gripper_cone_width), 0]';
gripper_cone_left  = [-sin(gripper_cone_width), cos(gripper_cone_width), 0]';

if dir_rot == 1
    init_roll_ang = angBTVec(init, gripper_cone_right, -z, 1);
else
    init_roll_ang = angBTVec(init, gripper_cone_left, z, 1);
end

% calculate goal angle
ang2goal = 0;
if dir_rot > 0
    ang2goal_f = angBTVec(gripper_cone_right,  goal, z, 1); % to further edge
    ang2goal_i = angBTVec(gripper_cone_left,  goal, z, 1);  % to closer edge
else % dir_rot < 0
    ang2goal_f = angBTVec(gripper_cone_left,  goal, -z, 1);
    ang2goal_i = angBTVec(gripper_cone_right,  goal, -z, 1);
end

if ang2goal_f < ang2goal_i
    ang2goal_i = ang2goal_i - 2*pi;
end

% initialization
s_ps     = ps;
s_com    = com;
s_goal   = goal;
s_angle  = 0;

ang2next        = para.PIVOTABLE_CHECK_GRANULARITY;

Nframes = floor(ang2goal_f/para.PIVOTABLE_CHECK_GRANULARITY)+1;
motion  = zeros(1, Nframes);
sliding = true(1, Nframes);
rtype   = zeros(1, Nframes);

axisz   = -z*dir_rot;
R1      = aa2mat(para.PIVOTABLE_CHECK_GRANULARITY, axisz);

% begin rotation
for fr = 1:Nframes
    % check how much do we need to rotate... can we reach the goal?
    if ang2goal_f < s_angle + para.PIVOTABLE_CHECK_GRANULARITY
        assert(fr == Nframes);
        ang2next        = ang2goal_f - s_angle;
        Ri              = aa2mat(ang2next, axisz);
    else
        Ri = R1;
    end
    
    % do the rotation
    s_com   = Ri*s_com;
    s_goal  = Ri*s_goal;
    s_ps    = Ri*s_ps;
    s_angle = s_angle + ang2next;

    % find out new contact points
    s_ps_w              = Rw_2d'*s_ps;
    [psz_min, z_min_id] = min(s_ps_w(3, :));
    assert(psz_min < 0);
    s_cpid              = s_ps_w(3, :) - psz_min < 2*ps_err + 1e-4;
    s_cp                = s_ps(:, s_cpid);

    % check current rotation state
    rtype(fr)  = checkPivotability(s_com, s_ps(:, z_min_id), min(s_cp(1,:)), max(s_cp(1,:)), gp, g, ps_err, para);
    motion(fr) = ang2next;
    assert(ang2next >= 0);
    if rtype(fr) == -1
        % constraint violated
        if ang2goal > 0
            % already good enough
            fr = fr-1;
            break;
        else
            flag = -1; % flag could be re-written later, if success
            plan = [];
            return; 
        end
    elseif rtype(fr) == 1 
        % pivoting
        if s_ps(1, z_min_id)*dir_rot <= 0
            % going down
            % find the smallest possible angle
            assert(~any(s_cp(1, :)*dir_rot > 0)); % all possible contact points should be in one quadrant 
            cpid_dangerous = s_cp(2, :) < 0; % only those potential contacts need to be considered
            cones = atan2(abs(s_cp(1, cpid_dangerous)), -s_cp(2, cpid_dangerous));
            if min(cones) < CONE
                sliding(fr) = false;
            end
        end
    end

    % check if in goal region
    if (ang2goal_i < s_angle)
        ang2goal = s_angle;
    end

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
        if rtype(fr) == 1
            h_fingertip.Color = [0 1 1];
        else
            h_fingertip.Color = [0.5 0.5 0];
        end
        axis equal
        drawnow
    end
end % end all frames

assert(~any(motion < 0));
flag          = 1; % successful
plan.motion   = motion(1:fr);
plan.rtype    = rtype(1:fr);
plan.sliding  = sliding(1:fr);
plan.ang2edge = ang2goal_f - ang2goal;
assert(abs(sum(plan.motion)-ang2goal) < 1e-5);
end





% Inputs
%   cp_l, cp_r: range of possible contact points
% Outputs
%   pivotable
%     1: can do pivot
%     0: can do roll
%    -1: infeasible
function pivotable = checkPivotability(com, cp, cp_l, cp_r, gp, g, ps_err, para)
	% 0: only roll
    pivotable = 0;
    % 1: can pivot
	if (cp_l*com(1) > 0) && (cp_r*com(1) > 0)
		pivotable = 1;
	end
    % -1: gripper limit violated
	if ((cp-gp(:,1))'*g < para.GRIPPER_Z_LIMIT + ps_err)||((cp-gp(:,2))'*g < para.GRIPPER_Z_LIMIT + ps_err)
		pivotable = -1;
	end

end
