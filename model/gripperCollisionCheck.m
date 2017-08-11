% Start from fingertips
% If fingertips collide, feasible_range = []
% Otherwise, check the finger&palm with different angles and store in feasible_range
% Angles are measured w.r.t. the "GetProperGrasp' angle at original grasp position
% output
% 	feasible_range: 360x1

function [feasible_range] = gripperCollisionCheck(mesh, mesh_s, gripper, gp)
DELTA = 1e-3;
feasible_range = [];

% ----------------------------------------------
% 	Check fingertip
% 	Use full mesh
% ----------------------------------------------
% Check fingertips, regardless of object orientation
qgrasp = getProperGrasp(gp(:,1), gp(:,2)); 

% open finger a little big when checking fingertip
ax             = quatOnVec([1 0 0]', qgrasp); ax = ax/norm(ax);
gp_delta       = gp;
gp_delta(:, 1) = gp(:, 1) + (DELTA + 0.01)*ax;
gp_delta(:, 2) = gp(:, 2) - (DELTA + 0.01)*ax;

% figure(1);clf(1);
% plotObject(mesh, 1);
% plotGripper(1, gripper, [1 0 0 0]', gp, qgrasp);

fingertip_plus.vertices  = bsxfun(@plus, quatOnVec(gripper.vertices{1}, qgrasp), gp_delta(:, 1))';
fingertip_minus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices{2}, qgrasp), gp_delta(:, 2))';
fingertip_plus.faces     = gripper.faces{1};
fingertip_minus.faces    = gripper.faces{2};

% check with coarse model
[~, surf] = SurfaceIntersection(mesh_s, fingertip_plus, 'debug', false);
if checkResult(surf)
	return;
end
[~, surf] = SurfaceIntersection(mesh_s, fingertip_minus, 'debug', false);
if checkResult(surf)
	return;
end

gp_delta(:, 1) = gp(:, 1) + DELTA*ax;
gp_delta(:, 2) = gp(:, 2) - DELTA*ax;
fingertip_plus.vertices  = bsxfun(@plus, quatOnVec(gripper.vertices{1}, qgrasp), gp_delta(:, 1))';
fingertip_minus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices{2}, qgrasp), gp_delta(:, 2))';
% check with precise model
[~, surf] = SurfaceIntersection(mesh, fingertip_plus, 'debug', false);
if checkResult(surf)
	return;
end
[~, surf] = SurfaceIntersection(mesh, fingertip_minus, 'debug', false);
if checkResult(surf)
	return;
end

% ----------------------------------------------
% 	Check Angles
% 	Use simplified object mesh
% ----------------------------------------------
% check angles
theta = [0:359] * pi/180;
q     = aa2quat(theta, ax);
gq    = quatMTimes(q, qgrasp);

feasible_range     = false(360, 1);
finger_plus.faces  = gripper.faces{3};
finger_minus.faces = gripper.faces{4};
palm.faces         = gripper.faces{5};

for i = 1:360
	feasible_range(i) = true;

	finger_plus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices{3}, gq(:, i)), gp(:,1))';
	[~, surf]            = SurfaceIntersection(mesh_s, finger_plus, 'debug', false);
	if checkResult(surf)
		feasible_range(i) = false;
		continue;
	end	

	finger_minus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices{4}, gq(:, i)), gp(:,2))';
	[~, surf]             = SurfaceIntersection(mesh_s, finger_minus, 'debug', false);
	if checkResult(surf)
		feasible_range(i) = false;
		continue;
	end	

	palm_plus     = bsxfun(@plus, quatOnVec(gripper.vertices{5}, gq(:, i)), gp(:,1));
	palm_minus    = bsxfun(@plus, quatOnVec(gripper.vertices{6}, gq(:, i)), gp(:,2));
	palm.vertices = [palm_plus palm_minus]';
	[~, surf]     = SurfaceIntersection(mesh_s, palm, 'debug', false);
	if checkResult(surf)
		feasible_range(i) = false;
		continue;
	end	
end % end for

% reject small range grasp points
if sum(feasible_range) < 10
	feasible_range = [];
end

end % end function



function has_edge = checkResult(surf)
if isempty(surf.faces)
    has_edge = false;
elseif any(surf.faces(:,2)-surf.faces(:,3) == 0)
    has_edge = true;
else
    has_edge = false;
end
end