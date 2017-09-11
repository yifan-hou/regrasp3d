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
function [qg, qframe, id_center, range_] = getProperGrasp(gp1, gp2, range0, q0, gp10, gp20, para, gripper_cone_width)
v  = gp1 - gp2;
v  = v/norm(v);

q1 = quatBTVec([1 0 0]', v);

if abs(v(3)) < 1e-5
    qg = q1;
else
    u1 = quatOnVec([0 1 0]', q1);
    % u  = cross(v, [0 0 1]'); % wrong
    u  = cross([0 0 1]', v);

    if norm(u) < 1e-5
        % grasp axis is vertical....
        % qg should get rejected in planning anyway
        qg = q1;
    else
        % get the rotation that rotates u1 to u
        % then apply it to q1
        qtemp = quatBTVec(u1, u);
        qg    = quatMTimes(qtemp, q1);

        % check, there are two u, one of them will turn gravity upwards 
        g1 = quatOnVec([0 0 -1]', qg);
        if g1(3) > 0
        	% redo with -u
            u     = -u;
            qtemp = quatBTVec(u1, u);
            qg    = quatMTimes(qtemp, q1);
        end
    end
    
end

% % check:
% v
% quatOnVec([1 0 0]', qg)
qframe = qg;

if nargin < 3
    return;
end


% ------------------------------------------------
%   Collision checking
% ------------------------------------------------
% 1. calculate reference grasp in original pose
qg0 = getProperGrasp(gp10, gp20);

% 2. rotate reference grasp to current frame
qg0 = quatMTimes(q0, qg0);
assert(angBTVec(quatOnVec([1 0 0]', qg0), v) < 1e-3);

% 3. compare and find the angle 
z0        = quatOnVec([0 0 1]', qg0);
z1        = quatOnVec([0 0 1]', qg);
ang       = round(angBTVec(z0, z1, v, 1)*180/pi); % 0~360
id_center = ang;
safe_zone = (ang - para.COLLISION_FREE_ANGLE_MARGIN):(ang + para.COLLISION_FREE_ANGLE_MARGIN);

if nargin >= 8
    % range 1: ok 0: collision
    gripper_cone_width_deg = round(gripper_cone_width*180/pi);
    cone_zone              = (ang - gripper_cone_width_deg):(ang + gripper_cone_width_deg);
    range0_zone            = circQuery(range0, cone_zone);
    range_                 = 0*range0;
    range_                 = circQuery(range_, cone_zone, range0_zone);
end


% 4. check collision, find a collision free solution
% move towards positive
displace_pos = 0;
while displace_pos < 180
    safe_zone_displaced = safe_zone + displace_pos;
    value_in_zone       = circQuery(range0, safe_zone_displaced);
    if ~any( value_in_zone == 0)
        % no collision!
        break;
    end
    id_bad = find(value_in_zone == 0);
    displace_pos = displace_pos + max(id_bad);
end

if displace_pos == 0
    return;
end

% move towards negative
displace_neg = 0;
while displace_neg > -180
    safe_zone_displaced = safe_zone + displace_neg;
    value_in_zone       = circQuery(range0, safe_zone_displaced);
    if ~any( value_in_zone == 0)
        % no collision!
        break;
    end
    id_bad       = find(value_in_zone == 0);
    displace_neg = displace_neg - (length(safe_zone) - min(id_bad) + 1);
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