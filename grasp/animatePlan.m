% gp: grasp points in world frame
% q: 4x1
function animatePlan(mesh, grasps, gripper, fidOrhandle, path_q, path_graspid, path_qp, path_gripper_plan_2d)

STEP_LENGTH_RAD = 3*pi/180;
% STEP_LENGTH_METER = 0.1;
% --------------------------------------
% 	draw and get the handles
% --------------------------------------
% object states
q0        = path_q(:,1);
m0_o      = quat2m(q0);
com_w     = m0_o*mesh.COM;
points_w  = m0_o*(mesh.vertices');

% gripper states
gp1o_w          = grasps.points(:,path_graspid(1), 1);
gp2o_w          = grasps.points(:,path_graspid(1), 2);
gp10_w          = quatOnVec(gp1o_w, q0);
gp20_w          = quatOnVec(gp2o_w, q0);
qgrasp0_w       = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate

if isnumeric(fidOrhandle)
	figure(fidOrhandle);
    clf;
else
	axes(fidOrhandle);
    cla;
end

% Initial plotting, get handles,
Rgrp = quat2m(qgrasp0_w);
fingertip_plus  = bsxfun(@plus, Rgrp*gripper.vertices{1}, gp10_w);
fingertip_minus = bsxfun(@plus, Rgrp*gripper.vertices{2}, gp20_w);
finger_plus     = bsxfun(@plus, Rgrp*gripper.vertices{3}, gp10_w);
finger_minus    = bsxfun(@plus, Rgrp*gripper.vertices{4}, gp20_w);
palm_plus       = bsxfun(@plus, Rgrp*gripper.vertices{5}, gp10_w);
palm_minus      = bsxfun(@plus, Rgrp*gripper.vertices{6}, gp20_w);
palm            = [palm_plus palm_minus];

[~, cpid] = min(points_w(3,:));
cpid      = abs(points_w(3,:) - points_w(3,cpid))<1e-5;
cp_w      = points_w(:,cpid);

% new plot
hold on;
% object
h_vertices = plot3(points_w(1,:), points_w(2,:), points_w(3,:), '.');
h_cp	   = plot3(cp_w(1,:), cp_w(2,:), cp_w(3,:), '.k', 'markersize', 30);
h_com	   = plot3(com_w(1), com_w(2), com_w(3), 'r*', 'markersize', 8);
h_gravity  = plot3(com_w(1)+[0 0], com_w(2)+[0 0], com_w(3)+[0 -0.4], 'r-', 'linewidth', 2);
h_surf     = patch('Faces',           mesh.faces,   ...
				   'Vertices',        points_w',  ...
				   'FaceColor',       [0.8 0.8 1.0], ...
			       'EdgeColor',       'none',        ...
			       'FaceLighting',    'gouraud',     ...
			       'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
h_fingertip_plus  = trisurf(gripper.faces{1}, fingertip_plus(1,:), fingertip_plus(2,:), fingertip_plus(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.9, 'linewidth', 0.1);
h_fingertip_minus = trisurf(gripper.faces{2}, fingertip_minus(1,:), fingertip_minus(2,:), fingertip_minus(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.9, 'linewidth', 0.1);
h_finger_plus     = trisurf(gripper.faces{3}, finger_plus(1,:), finger_plus(2,:), finger_plus(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.5);
h_finger_minus    = trisurf(gripper.faces{4}, finger_minus(1,:), finger_minus(2,:), finger_minus(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.5);
h_palm            = trisurf(gripper.faces{5}, palm(1,:), palm(2,:), palm(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.7);

xlabel('X'); ylabel('Y'); zlabel('Z');
view(-43, 27);


NP = size(path_q, 2);
for p = 1:NP-1
	q0       = path_q(:,p);
	qf       = path_q(:,p+1);
	qp       = path_qp(:,p);
	m0_o     = quat2m(q0);
	com_w    = m0_o*mesh.COM;
	points_w = m0_o*(mesh.vertices');

	% grasp
	gp1o_w    = grasps.points(:,path_graspid(1), 1);
	gp2o_w    = grasps.points(:,path_graspid(1), 2);
	gp10_w    = quatOnVec(gp1o_w, q0);
	gp20_w    = quatOnVec(gp2o_w, q0);
	qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate

	pause(0.5);
	% First plot for one transition.
	% Update position of gripper
	
	updatePlot(gripper, eye(3), eye(3), gp10_w, gp20_w, qgrasp0_w, com_w, points_w,...
			   h_vertices, h_cp, h_com, h_gravity, h_surf, h_fingertip_plus, h_fingertip_minus, ...
			   h_finger_plus, h_finger_minus, h_palm);

	% --------------------------------------
	% 	Pre-Rolling to qp
	% --------------------------------------
	num = floor(angBTquat(q0, qp)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(q0, qp, t);

	h_fingertip_plus.FaceColor  = [0.9 0.1 0];
	h_fingertip_minus.FaceColor = [0.9 0.1 0];

	for s = 1:length(t)
		q0i_w = quatMTimes(qi(:,s), quatInv(q0)); % a rotation that rotates q0 to qi(:,s), measured in world frame
		Robj  = quat2m(q0i_w);
		Rgrp  = Robj;
		updatePlot(gripper, Robj, Rgrp, gp10_w, gp20_w, qgrasp0_w, com_w, points_w,...
				   h_vertices, h_cp, h_com, h_gravity, h_surf, h_fingertip_plus, h_fingertip_minus, ...
			       h_finger_plus, h_finger_minus, h_palm);
	end

	% update states
	[gp10_w, gp20_w, com_w, points_w, qgrasp0_w] = updateStates(gp10_w, gp20_w, com_w, points_w, qgrasp0_w, Robj, Rgrp);
    % object: qp now
    q_now = qp;
    
	% --------------------------------------
	% 	Pivoting
	% --------------------------------------
	ang_obj = 0;
	ang_grp = 0;
	n       = gp10_w - gp20_w; n = n/norm(n); % rotation axis
	if path_gripper_plan_2d{p}.dir > 0
		n = -n;
	end

	for s = 1:length(path_gripper_plan_2d{p}.gripper_motion)
		ang_obj_incre = path_gripper_plan_2d{p}.object_motion(s);
		ang_grp_incre = path_gripper_plan_2d{p}.gripper_motion(s);
		assert(~any(ang_obj_incre < 0)); 

		num = floor(max(ang_obj_incre, ang_grp_incre)/STEP_LENGTH_RAD);
		if num == 0
			num = 1;
		end

		ang_obj_incre_array = fixStepSample(0, ang_obj_incre, num);
		ang_grp_incre_array = fixStepSample(0, ang_grp_incre, num);

		if(path_gripper_plan_2d{p}.rtype(s) == 0)
			h_fingertip_plus.FaceColor  = [0.9 0.1 0];
			h_fingertip_minus.FaceColor = [0.9 0.1 0];
		else
			h_fingertip_plus.FaceColor  = [0.1 0.0 0.9];
			h_fingertip_minus.FaceColor = [0.1 0.0 0.9];
		end

		for i = 1:length(ang_obj_incre_array)	
			% calculate
			Robj = aa2mat(ang_obj + ang_obj_incre_array(i), n);
			Rgrp = aa2mat(ang_grp + ang_grp_incre_array(i), n);

			updatePlot(gripper, Robj, Rgrp, gp10_w, gp20_w, qgrasp0_w, com_w, points_w,...
				   h_vertices, h_cp, h_com, h_gravity, h_surf, h_fingertip_plus, h_fingertip_minus, ...
				   h_finger_plus, h_finger_minus, h_palm);
		end
		ang_obj = ang_obj + ang_obj_incre;
		ang_grp = ang_grp + ang_grp_incre;

	end

	% update states
	[gp10_w, gp20_w, com_w, points_w, qgrasp0_w] = updateStates(gp10_w, gp20_w, com_w, points_w, qgrasp0_w, Robj, Rgrp);
    q_now = quatMTimes(mat2quat(Robj), q_now);
	% --------------------------------------
	% 	Post-Rolling
    %   From q_now to qf
	% --------------------------------------
	num = floor(angBTquat(q_now, qf)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(q_now, qf, t);

	h_fingertip_plus.FaceColor  = [0.9 0.1 0];
	h_fingertip_minus.FaceColor = [0.9 0.1 0];

	for s = 1:length(t)
		q0i_w = quatMTimes(qi(:,s), quatInv(q_now)); % a rotation that rotates q_now to qi(:,s), measured in world frame
		Robj  = quat2m(q0i_w);
		Rgrp  = Robj;
		updatePlot(gripper, Robj, Rgrp, gp10_w, gp20_w, qgrasp0_w, com_w, points_w,...
				   h_vertices, h_cp, h_com, h_gravity, h_surf, h_fingertip_plus, h_fingertip_minus, ...
			       h_finger_plus, h_finger_minus, h_palm);
	end

end % finish the whole path


% h = plot3()
% Line with properties:

%             Color: [1 0 0]
%         LineStyle: '-'
%         LineWidth: 0.5000
%            Marker: '*'
%        MarkerSize: 5
%   MarkerFaceColor: 'none'
%             XData: 0
%             YData: 0
%             ZData: 0

end



function updatePlot(gripper, Robj, Rgrp, gp1, gp2, qgp0, com, points, h_vertices, h_cp, h_com, h_gravity, h_surf, h_fingertip_plus, h_fingertip_minus, h_finger_plus, h_finger_minus, h_palm)
	gp1_ = Robj*gp1;
	gp2_ = Robj*gp2;
	com_ = Robj*com;
	ps_  = Robj*points;

	Rgrp = Rgrp*quat2m(qgp0);
    
	fingertip_plus  = bsxfun(@plus, Rgrp*gripper.vertices{1}, gp1_);
	fingertip_minus = bsxfun(@plus, Rgrp*gripper.vertices{2}, gp2_);
	finger_plus     = bsxfun(@plus, Rgrp*gripper.vertices{3}, gp1_);
	finger_minus    = bsxfun(@plus, Rgrp*gripper.vertices{4}, gp2_);
	palm_plus       = bsxfun(@plus, Rgrp*gripper.vertices{5}, gp1_);
	palm_minus      = bsxfun(@plus, Rgrp*gripper.vertices{6}, gp2_);
	palm            = [palm_plus palm_minus];

	[~, cpid] = min(ps_(3,:));
	cpid      = abs(ps_(3,:) - ps_(3,cpid))<1e-5;
	cp_       = ps_(:,cpid);

	% update plot
	h_vertices.XData  = ps_(1,:);
	h_vertices.YData  = ps_(2,:);
	h_vertices.ZData  = ps_(3,:);
	h_cp.XData        = cp_(1,:);
	h_cp.YData        = cp_(2,:);
	h_cp.ZData        = cp_(3,:);
	h_com.XData       = com_(1);
	h_com.YData       = com_(2);
	h_com.ZData       = com_(3);
	h_gravity.XData   = com_(1)+[0  0]; 
	h_gravity.YData   = com_(2)+[0  0]; 
	h_gravity.ZData   = com_(3)+[0 -1];

	h_surf.Vertices   = ps_';

	h_fingertip_plus.Vertices  = fingertip_plus';
	h_fingertip_minus.Vertices = fingertip_minus';
	h_finger_plus.Vertices     = finger_plus';
	h_finger_minus.Vertices    = finger_minus';
	h_palm.Vertices            = palm';
	 

	axis([com_(1)-1 com_(1)+1 com_(2)-1 com_(2)+1 com_(3)-1 com_(3)+1]);
	axis equal;
	drawnow;
end

function [gp1o_w, gp2o_w, com, points, qg] = updateStates(gp1o_w, gp2o_w, com, points, qg, Robj, Rgrp)
	% update states
	gp1o_w = Robj*gp1o_w;
	gp2o_w = Robj*gp2o_w;
	com    = Robj*com;
	points = Robj*points;
	qg     = quatMTimes(mat2quat(Rgrp), qg);
end