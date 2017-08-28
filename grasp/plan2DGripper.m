% outputs:
% 	plan.obj_motion_diff:  1xNc array of object rotation angles in each of N stages. rotation w.r.t [0 0 1]' in grasp frame.
% 	plan.grp_motion_diff: 1xNc array of gripper rotation angles in each of N stages. rotation w.r.t [0 0 1]' in grasp frame.
% 	plan.grp_init_ang: initial gripper ang. 0 is in middle.
% 	plan.rtype: 1xNc array
% 	plan.dir: scalar
function	plan = plan2DGripper(object_plan, init_grasp_ang, gripper_cone_width)
N  = length(object_plan.motion);
% Nr = length(object_plan.cf_range_at_roll);

grp_motion_diff = zeros(1, N); % gripper relative motion in each stage
grp_motion_acc  = init_grasp_ang + gripper_cone_width; % 0: rightmost edge of gripper range

% object total rotation angle
obj_ending_2 = sum(object_plan.motion);
obj_ending_1 = obj_ending_2 - 2*gripper_cone_width;

roll_count   = 0;
mid_range_id = round(size(object_plan.cf_range_at_roll, 1)/2);
for s = 1:N
	% stage s

	obj_motion_acc     = sum(object_plan.motion(1:(s-1)));
	obj_motion_acc_nxt = sum(object_plan.motion(1:s    ));

	if object_plan.rtype(s) == 0
		% Now is a roll
		% check termination
		if obj_motion_acc_nxt + 1e-5 > obj_ending_2

			% terminate! check success
			if object_plan.dir > 0
				if abs(grp_motion_acc - (obj_ending_2 - obj_motion_acc)) < 1e-3
					break;
				end
			else
				if abs(2*gripper_cone_width - grp_motion_acc - (obj_ending_2 - obj_motion_acc)) < 1e-3
					break;
				end
			end
			% fail
			plan = [];
			return;
		end

		grp_motion_diff(s) = -object_plan.dir*object_plan.motion(s);
		grp_motion_acc     = grp_motion_acc + grp_motion_diff(s);
		roll_count         = roll_count + 1;
	else
		% Now is a pivot
		% check termination
		if obj_motion_acc_nxt > obj_ending_1

			% terminate!
			if obj_motion_acc > obj_ending_1
				% no object rotation is needed, just rotate the gripper
				object_plan.motion(s) = 0; 
				if object_plan.dir > 0
					grp_motion_diff(s) = gripper_cone_width*2 - grp_motion_acc - (obj_motion_acc - obj_ending_1); %ok
				else
					grp_motion_diff(s) =  (obj_motion_acc - obj_ending_1) - grp_motion_acc;
				end
			else
				object_plan.motion(s) = obj_ending_1 - obj_motion_acc;
				if object_plan.dir > 0
					grp_motion_diff(s) = gripper_cone_width*2 - grp_motion_acc;
				else
					grp_motion_diff(s) =  - grp_motion_acc;
				end
			end

			grp_motion_acc = grp_motion_acc + grp_motion_diff(s);
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

plan.rtype           = object_plan.rtype(1:s);
plan.dir             = object_plan.dir;
plan.obj_motion_diff = object_plan.motion(1:s);
plan.grp_motion_diff = grp_motion_diff(1:s);
plan.grp_init_ang    = init_grasp_ang;

% check
final_grasp_ang = init_grasp_ang + sum(plan.grp_motion_diff);
disp(['final_grasp_ang: ' num2str(180/pi*final_grasp_ang)]);

end