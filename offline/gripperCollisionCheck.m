% Start from fingertips
% If fingertips collide with object, feasible_range = []
% Otherwise, check the finger&palm with different angles and store in feasible_range
% Angles are measured w.r.t. the "GetProperGrasp' angle at original grasp position
% output
% 	feasible_range: 360x1.   1: collision free. 0: has collision
% 	qgrasp:	4x1, a quaternion frame. x axis: gp(:,1) - gp(:,2), z axis: theta = 0
function [feasible_range, qgrasp] = gripperCollisionCheck(mesh, mesh_s, gripper, gp, para)

feasible_range = [];

% ----------------------------------------------
% 	Check fingertip
% 	Use full mesh
% ----------------------------------------------
% Check fingertips, regardless of object orientation
if norm(gp(:,1) - gp(:,2)) < 1e-3
    qgrasp = [0 0 0 0]';
    return;
end

qgrasp = getProperGraspSimple(gp(:,1), gp(:,2)); 

% open finger a little big when checking fingertip,
% in case the grasp is tilting
space              = 3; %mm
ax                 = quatOnVec([1 0 0]', qgrasp); ax = ax/norm(ax);
gp_delta           = gp;
gp_delta(:, 1)     = gp(:, 1) + space*ax;
gp_delta(:, 2)     = gp(:, 2) - space*ax;

figure(1);clf(1);
plotObject(mesh, 1);
plotGripper(1, gripper, [1 0 0 0]', gp, qgrasp);

fingertip_plus.vertices  = bsxfun(@plus, quatOnVec(gripper.vertices_safe{1}, qgrasp), gp_delta(:, 1))';
fingertip_minus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices_safe{2}, qgrasp), gp_delta(:, 2))';
fingertip_plus.faces     = gripper.faces{1};
fingertip_minus.faces    = gripper.faces{2};


% check with precise model
[~, surf] = SurfaceIntersection(mesh_s, fingertip_plus, 'debug', false);
if checkResult(surf)
	return;
end
[~, surf] = SurfaceIntersection(mesh_s, fingertip_minus, 'debug', false);
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

feasible_range     = false(1, 360);
finger_plus.faces  = gripper.faces{3};
finger_minus.faces = gripper.faces{4};
palm.faces         = gripper.faces{5};

for i = 1:360
	feasible_range(i) = true;

	finger_plus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices_safe{3}, gq(:, i)), gp(:,1))';
	[~, surf]            = SurfaceIntersection(mesh_s, finger_plus, 'debug', false);
	if checkResult(surf)
		feasible_range(i) = false;
		continue;
	end	

	finger_minus.vertices = bsxfun(@plus, quatOnVec(gripper.vertices_safe{4}, gq(:, i)), gp(:,2))';
	[~, surf]             = SurfaceIntersection(mesh_s, finger_minus, 'debug', false);
	if checkResult(surf)
		feasible_range(i) = false;
		continue;
	end	

	palm_plus     = bsxfun(@plus, quatOnVec(gripper.vertices_safe{5}, gq(:, i)), gp(:,1));
	palm_minus    = bsxfun(@plus, quatOnVec(gripper.vertices_safe{6}, gq(:, i)), gp(:,2));
	palm.vertices = [palm_plus palm_minus]';
	[~, surf]     = SurfaceIntersection(mesh_s, palm, 'debug', false);
	if checkResult(surf)
		feasible_range(i) = false;
		continue;
	end	
end % end for

% shrink feasible_range for robustness 
start1      = strfind([0, feasible_range==1],[0 1]);
end1        = strfind([feasible_range==1,0],[1 0]);
length_of_1 = end1 - start1 + 1;

for i = 1:length(length_of_1)
	feasible_range = circQuery(feasible_range, start1:start1+para.COLLISION_FREE_ANGLE_MARGIN-1, 0);
	feasible_range = circQuery(feasible_range, end1-para.COLLISION_FREE_ANGLE_MARGIN+1:end1, 0);
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