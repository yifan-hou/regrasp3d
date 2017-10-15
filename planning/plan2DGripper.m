% outputs:
% 	plan.obj_motion_diff:  1xNc array of object rotation angles in each of N stages. rotation w.r.t [0 0 1]' in grasp frame.
% 	plan.grp_motion_diff: 1xNc array of gripper rotation angles in each of N stages. rotation w.r.t [0 0 1]' in grasp frame.
% 	plan.rtype: 1xNc array
% 	plan.dir: scalar
function	plan = plan2DGripper(object_plan, init_grasp_ang, gripper_cone_width)
N  = length(object_plan.motion);

grp_motion_diff = zeros(1, N); % gripper relative motion in each stage
grp_motion_acc  = init_grasp_ang + gripper_cone_width; % 0: rightmost edge of gripper range

% object total rotation angle
obj_ending_2 = sum(object_plan.motion); % might be on further edge
obj_ending_1 = obj_ending_2 + object_plan.ang2edge - 2*gripper_cone_width; % on proximal edge, could be negative

% --------------------------------
% 	Choose end point
% --------------------------------
obj_ending = obj_ending_1 + gripper_cone_width; % the ideal ending position: gripper at middle
if obj_ending < 0
    obj_ending = 0;
end

for s = 1:N
	obj_motion_acc     = sum(object_plan.motion(1:(s-1)));
	obj_motion_acc_nxt = sum(object_plan.motion(1:s    ));

	if obj_motion_acc_nxt + 1e-5 > obj_ending_1
		% time to determine the ending !!
		if object_plan.rtype(s) == 1
			% good, just use the obj_ending
			obj_ending = obj_ending_1;
		else 
			% use next stage
			obj_ending = obj_motion_acc_nxt;
			assert(obj_ending < obj_ending_2 + 1e-5);
		end
		break;
	end
end % end for

if obj_ending < 0
    obj_ending = 0;
end

% --------------------------------
% 	Calculate gripper motions
% --------------------------------

roll_count   = 0;
mid_range_id = round(size(object_plan.cf_range_at_roll, 1)/2);
for s = 1:N
	% stage s
	obj_motion_acc     = sum(object_plan.motion(1:(s-1)));
	obj_motion_acc_nxt = sum(object_plan.motion(1:s    ));

	if object_plan.rtype(s) == 0
		% Now is a roll
		grp_motion_diff(s) = -object_plan.dir*object_plan.motion(s);
		grp_motion_acc     = grp_motion_acc + grp_motion_diff(s);
		roll_count         = roll_count + 1;
		if s == N
			plan = [];
			return;
		end
	else
		% Now is a pivot
		% check termination
		if obj_motion_acc_nxt + 1e-5 > obj_ending
			object_plan.motion(s) = obj_ending - obj_motion_acc;
			assert(object_plan.motion(s) >= 0);

			if object_plan.dir > 0
				grp_ending = obj_ending_2 + object_plan.ang2edge - obj_ending;
			else
				grp_ending = obj_ending - obj_ending_1;
			end
			grp_motion_diff(s) = grp_ending - grp_motion_acc;
			grp_motion_acc     = grp_ending;	
			break;
		end

		% calculate gripper motion
		feasible_id        = find(object_plan.cf_range_at_roll(:, roll_count+1));
		[~, id]            = min(abs(feasible_id-mid_range_id));
		grp_motion_acc_id  = round(180/pi*(grp_motion_acc)) + 1; % 1: rightmost edge of gripper range
		grp_motion_diff(s) = (feasible_id(id) - grp_motion_acc_id)*pi/180;
		grp_motion_acc     = grp_motion_acc + grp_motion_diff(s);
	end

end

plan.obj_sliding_acc  = object_plan.sliding_motion;
plan.obj_sliding_type = object_plan.sliding;
plan.rtype            = object_plan.rtype(1:s);
plan.dir              = object_plan.dir;
plan.obj_motion_diff  = object_plan.motion(1:s);
plan.grp_motion_diff  = grp_motion_diff(1:s);

end