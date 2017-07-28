% outputs:
% 	plan.object:  1xNc array of object rotation angles in each of N stages. rotation w.r.t [0 0 1]' in grasp frame.
% 	plan.gripper: 1xNc array of gripper rotation angles in each of N stages. rotation w.r.t [0 0 1]' in grasp frame.
% 	plan.rtype: 1xNc array
% 	plan.dir: scalar
function	plan = plan2DGripper(object_plan, init_grasp_ang, para)
object_motion = object_plan.motion;
object_rtype  = object_plan.rtype;
object_dir    = object_plan.dir;

N = length(object_motion);

% --------------------------------------------------
% 	Calculate consecutive object motion, 
% 	combine adjacent same type motions
% --------------------------------------------------
object_rtype = [object_rtype 999];
object_motion_consec = zeros(1, N);
object_rtype_consec  = zeros(1, N);
pt_consec = 1;
pt = 1;
while true
	next_angle = object_motion(pt);
	next_rtype = object_rtype(pt);
	while object_rtype(pt+1) == object_rtype(pt)
		next_angle = next_angle + object_motion(pt+1);
		pt = pt + 1;
		if pt == N
			break;
		end
	end
	object_motion_consec(pt_consec) = next_angle;
	object_rtype_consec(pt_consec)  = next_rtype;
	pt_consec = pt_consec + 1;

	if pt == N
		break;
	end
	pt = pt + 1;
end

object_motion_consec(pt_consec:end) = [];
object_rtype_consec(pt_consec:end)  = [];

% --------------------------------------------------
% 	figure out gripper rotation
% --------------------------------------------------
Nc                    = length(object_motion_consec);
gripper_motion_consec = zeros(1, Nc);
x                     = para.GRIPPER_TILT_LIMIT - init_grasp_ang;
assert(object_rtype_consec(end) == 1);
for i = 1:Nc
	if object_rtype_consec(i) == 1
		% pivoting
		if i < Nc
			% need to prepare for next rolling
			next_roll = object_motion_consec(i+1);
			if x > -0.5*next_roll
				gripper_motion_consec(i) = -0.5*next_roll - x;
			else
				gripper_motion_consec(i) = 0;
			end
		else
			% get to goal angle
			gripper_motion_consec(i) = -x;
		end
	else
		% rolling
		gripper_motion_consec(i) = object_motion_consec(i);
	end
	x = x + gripper_motion_consec(i);
end

plan.rtype          = object_rtype_consec;
plan.dir            = object_dir;
plan.object_motion  = object_motion_consec;
plan.gripper_motion = gripper_motion_consec;

end