% select the set of good grasps for a given pose
% check z limit, orientation limit
function [grasp_id] = checkGraspPoints(grasps, mesh, pgraph, q, fig_id, para)

% --------------------------------------------
% 		Parameters
% --------------------------------------------
GRIPPER_TILT_LIMIT = para.GRIPPER_TILT_LIMIT;
GRIPPER_Z_LIMIT    = para.GRIPPER_Z_LIMIT;
tilt_z_limit       = sin(GRIPPER_TILT_LIMIT);

grasp_id 		   = ones(1, grasps.count);

% find the bottom point, vertical offset
points_rot = zeros(3, pgraph.NPC);
for j = 1:pgraph.NPC
	points_rot(:,j) = quatOnVec(pgraph.vertices(:, j), q);
end
z_offset = min(points_rot(3,:)) + pgraph.err_bound;

% visualization
if para.showGraspChecking
	% object
	plotObject(mesh, fig_id, q); hold on;
end

for j = 1:grasps.count
	% rotate grasp
	p1 = quatOnVec(grasps.points(:,j,1), q);
	p2 = quatOnVec(grasps.points(:,j,2), q);

	% visualization
	if para.showGraspChecking
		% grasp point
		plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], '-b*','markersize',8, 'linewidth', 2);
	end



	% check z position limit
	if (p1(3) - z_offset < GRIPPER_Z_LIMIT) || (p2(3) - z_offset < GRIPPER_Z_LIMIT) 
		grasp_id(j) = 0;

		% visualization
		if para.showGraspChecking
			% grasp point
			if p1(3) - z_offset < GRIPPER_Z_LIMIT
				plot3(p1(1), p1(2), p1(3), 'ro','markersize',8);
			end
			if p2(3) - z_offset < GRIPPER_Z_LIMIT
				plot3(p2(1), p2(2), p2(3), 'ro','markersize',8);
			end
		end

		continue;
	end

	% check orientation limit
	b = p1 - p2; b = b/norm(b);
	if abs(b(3)) > tilt_z_limit
		grasp_id(j) = 0;

		% visualization
		if para.showGraspChecking
			% grasp point
			plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r-','linewidth',2);
		end

		continue;
	end

end



