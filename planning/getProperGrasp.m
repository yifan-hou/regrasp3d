% q moves [1 0 0] to (gp1 - gp2), [0 1 0] to [? ? 0]
% If given range0 and q, consider collision and output the 
% closest feasible solution
% Inputs:
%   gp1, gp2: 3x1, grasp pos in world frame
%   range0: 360x1, array of collision checking results in original frame
%   qg: pose of gp1, gp2 measured in world frame
% Outputs:
%   qg: a collision free orientation closed to upright
%   qframe: the grasp frame
%   id_center: id of current upright gripper direction in range0
%   range_: same as range0, but all the angle outside of gripper_cone_width are set to 0
function [qg, qframe, data] = getProperGrasp(grasp_id, qnow, gripper_cone_width, randgrasp)
global grasps

gp10 = grasps.points(:, grasp_id, 1);
gp20 = grasps.points(:, grasp_id, 2);

gp1 = quatOnVec(gp10, qnow);
gp2 = quatOnVec(gp20, qnow);

[qg, v] = getProperGraspSimple(gp1, gp2);
qframe  = qg;

% ------------------------------------------------
%   Find a feasible grasp pose 
%   as close to qg as possible
%       1. within cone width
%       2. collision free
% ------------------------------------------------
% 1. calculate reference grasp in original pose
qg0 = getProperGraspSimple(gp10, gp20);

% 2. rotate reference grasp to current frame
qg0 = quatMTimes(qnow, qg0);
assert(angBTVec(quatOnVec([1 0 0]', qg0), v) < 1e-3);

% 3. compare and find the angle 
z0        = quatOnVec([0 0 1]', qg0);
z1        = quatOnVec([0 0 1]', qg);
id_center = round(angBTVec(z0, z1, v, 1)*180/pi); % 0~360

% --------------------------------------
%   Get collision-free range within gripper cone width
% --------------------------------------

% range 1: ok 0: collision
gripper_cone_width_deg = round(gripper_cone_width*180/pi);

cone_zone   = (id_center - gripper_cone_width_deg):(id_center + gripper_cone_width_deg);
range0_zone = circQuery(grasps.range(:, grasp_id), cone_zone);
range_      = circQuery(zeros(size(grasps.range, 1), 1), cone_zone, range0_zone);

if nargout == 3
    % some temporary variables useful to pick&place
    data.id_center = id_center;
    data.qg0       = qg0;
    data.range     = range_;
    return;
end

id_range    = find(range_);
if isempty(id_range) 
    qg = [];
    return;
end


if randgrasp
    % pick one randomly
    while true
        id_rand   = id_range(randi(length(id_range)));
        if range_(id_rand)
            break;
        end
    end
    displace = (id_rand - id_center)*pi/180;
    qrot     = aa2quat(displace, v);
    qg       = quatMTimes(qrot, qg);
else
    % choose the one closest to qg

    % 4. check collision, find a collision free solution
    % move towards positive
    displace_pos = 0;
    while displace_pos < 180
        safe_zone_displaced = id_center + displace_pos;
        value_in_zone       = circQuery(range_, safe_zone_displaced);
        if value_in_zone
            % no collision!
            break;
        end
        displace_pos = displace_pos + 1;
    end

    if displace_pos == 0
        return;
    end

    % move towards negative
    displace_neg = 0;
    while displace_neg > -180
        safe_zone_displaced = id_center + displace_neg;
        value_in_zone       = circQuery(range_, safe_zone_displaced);
        if value_in_zone
            % no collision!
            break;
        end
        displace_neg = displace_neg - 1;
    end

    assert(min(displace_pos, abs(displace_neg)) < 180, 'This grasp has no solution!');

    % rotate the grasp frame
    if displace_pos < abs(displace_neg)
        displace = displace_pos*pi/180;
    else
        displace = displace_neg*pi/180;
    end

    qrot = aa2quat(displace, v);
    qg   = quatMTimes(qrot, qg);
end



end