
% combine adjacent motions with same type
% check roll angles, check collision.
% Input
%   motion: object motion at each stage. Nonnegative
% Output
%   feasible: bool
%   motion_: nx1 concatenated motion stages
%   type_: nx1 types of each stages, no consecutive value. 010101
%   sliding_motion: kx1 concatenated sliding stages. cumsum
%   sliding_: kx1 types of sliding states. no consecutive value. 010101
%   range_: 360xk, collision-free range at the beginning of each roll stage
function [plan2D] = check2DObjectTraj(plan_frames, init_roll_ang, dir_rot, cf_range, cf_zero_id, gripper_cone_width)

if isempty(plan_frames)
    plan2D = [];
    return;
end

N            = length(plan_frames.motion);
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
    accu_angle            = 0;
    type_(stage_count_+1) = plan_frames.rtype(i);
    while true
        accu_angle = accu_angle + plan_frames.motion(i);
        i = i + 1;
        if (i > N) || (plan_frames.rtype(i) ~= type_(stage_count_+1))
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
        zone_start             = cf_zero_id + round(180/pi*(dir_rot*sum(motion_) - gripper_cone_width));
        zone_end               = zone_start + Nrange - 1;
        zone                   = zone_start:zone_end;
        range_(:, roll_count_) = circQuery(cf_range, zone);
        if dir_rot
            range_(1:accu_angle_id, roll_count_) = false; 
        else
            range_(end-accu_angle_id:end, roll_count_) = false; 
        end

        if ~any(range_(:, roll_count_))
            feasible = false;
            break;
        end
    end

    first_roll              = false;
    motion_(stage_count_+1) = accu_angle;
    stage_count_            = stage_count_ + 1;

    if i > N
        break;
    end
end

if ~feasible
    plan2D = [];
    return;
end

motion_ = motion_(1:stage_count_);
type_   = type_(1:stage_count_);
range_  = range_(:, 1:roll_count_);

% ---------------------------------------
%   Check sliding state
% ---------------------------------------
sliding_        = zeros(N, 1);
sliding_motion_ = zeros(N, 1);
stage_count_    = 0;

i = 1;
while true
    accu_angle               = 0;
    sliding_(stage_count_+1) = plan_frames.sliding(i);
    while true
        accu_angle = accu_angle + plan_frames.motion(i);
        i = i + 1;
        if (i > N) || (plan_frames.sliding(i) ~= sliding_(stage_count_+1))
            break;
        end
    end

    sliding_motion_(stage_count_ + 1) = accu_angle;
    stage_count_                     = stage_count_ + 1;

    if i > N
        break;
    end
end

sliding_       = sliding_(1:stage_count_);
sliding_motion_ = cumsum(sliding_motion_(1:stage_count_));


plan2D.dir              = dir_rot;
plan2D.motion           = motion_;
plan2D.rtype            = type_;
plan2D.sliding          = sliding_;
plan2D.sliding_motion   = sliding_motion_;
plan2D.cf_range_at_roll = range_;
plan2D.ang2edge         = plan_frames.ang2edge;
end
