% gp: grasp points in world frame
% q: 4x1
function animatePlan(mesh, grasps, fidOrhandle, path_q, path_graspid, path_qp, path_gripper_plan_2d)

STEP_LENGTH_RAD = 3*pi/180;
% STEP_LENGTH_METER = 0.1;
% --------------------------------------
% 	draw and get the handles
% --------------------------------------
% object states
q0        = path_q(:,1);
m0_o      = quat2m(q0);
com_w     = m0_o*mesh.COM;
points_w  = m0_o*mesh.points;
[~, cpid] = min(points_w(3,:));
cpid      = find(abs(points_w(3,:) - points_w(3,cpid))<1e-5);
cp        = points_w(:,cpid);

% gripper states
gp1o_w    = grasps.points(:,path_graspid(1), 1);
gp2o_w    = grasps.points(:,path_graspid(1), 2);
gp10_w    = quatOnVec(gp1o_w, q0);
gp20_w    = quatOnVec(gp2o_w, q0);
qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate
dir       = quatOnVec([0 0 1]', qgrasp0_w);
gbase1    = gp10_w + 2*dir;
gbase2    = gp20_w + 2*dir;

if isnumeric(fidOrhandle)
	figure(fidOrhandle);
    clf;
else
	axes(fidOrhandle);
    cla;
end

hold on;
% object
h_vertices = plot3(points_w(1,:), points_w(2,:), points_w(3,:), '.');
h_cp	   = plot3(cp(1,:), cp(2,:), cp(3,:), '.k', 'markersize', 30);
h_com	   = plot3(com_w(1), com_w(2), com_w(3), 'r*', 'markersize', 8);
h_gravity  = plot3(com_w(1)+[0 0], com_w(2)+[0 0], com_w(3)+[0 -0.4], 'r-', 'linewidth', 2);
% h_surf     = trisurf(mesh.faces', points_w(1,:), points_w(2,:), points_w(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.3);
h_surf     = patch('Faces',           mesh.faces',   ...
				   'Vertices',        mesh.points',  ...
				   'FaceColor',       [0.8 0.8 1.0], ...
			       'EdgeColor',       'none',        ...
			       'FaceLighting',    'gouraud',     ...
			       'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
% fingertip
h_fingertip = plot3([gp10_w(1) gp20_w(1)], [gp10_w(2) gp20_w(2)], [gp10_w(3) gp20_w(3)], 'r.', 'markersize',40); 

% finger
h_gripper = plot3([gp10_w(1) gbase1(1) gbase2(1) gp20_w(1)],...
				  [gp10_w(2) gbase1(2) gbase2(2) gp20_w(2)],...
				  [gp10_w(3) gbase1(3) gbase2(3) gp20_w(3)],...
				  '-k','linewidth',8);

xlabel('X'); ylabel('Y'); zlabel('Z');
axis([com_w(1)-1 com_w(1)+1 com_w(2)-1 com_w(2)+1 com_w(3)-1 com_w(3)+1]);
axis equal;
view(-43, 27);


NP = size(path_q, 2);
for p = 1:NP-1
	q0 = path_q(:,p);
	qf = path_q(:,p+1);
	qp = path_qp(:,p);

	% grasp
	gp1o_w    = grasps.points(:,path_graspid(p), 1);
	gp2o_w    = grasps.points(:,path_graspid(p), 2);
	gp10_w    = quatOnVec(gp1o_w, q0);
	gp20_w    = quatOnVec(gp2o_w, q0);
	qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate
	dir       = quatOnVec([0 0 1]', qgrasp0_w);

	pause(0.5);
	updatePlot(eye(3), eye(3), gp10_w, gp20_w, com_w, points_w, dir, h_vertices, h_cp, h_com, h_gravity, h_fingertip, h_gripper, h_surf);

	% --------------------------------------
	% 	Pre-Rolling to qp
	% --------------------------------------
	num = floor(angBTquat(q0, qp)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(q0, qp, t);

	h_fingertip.Color      = [0.9 0.1 0];
	h_fingertip.MarkerSize = 40;

	for s = 1:length(t)
		q0i_w = quatMTimes(qi(:,s), quatInv(q0)); % a rotation that rotates q0 to qi(:,s), measured in world frame
		Robj  = quat2m(q0i_w);
		Rgrp  = Robj;
		updatePlot(Robj, Rgrp, gp10_w, gp20_w, com_w, points_w, dir, h_vertices, h_cp, h_com, h_gravity, h_fingertip, h_gripper, h_surf);
	end

	% update states
	[gp10_w, gp20_w, com_w, points_w, dir] = updateStates(gp10_w, gp20_w, com_w, points_w, dir, Robj, Rgrp);
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
			h_fingertip.Color      = [0.9 0.1 0];
			h_fingertip.MarkerSize = 40;
		else
			h_fingertip.Color      = [0.1 0 0.9];
			h_fingertip.MarkerSize = 40;
		end

		for i = 1:length(ang_obj_incre_array)	
			% calculate
			Robj = aa2mat(ang_obj + ang_obj_incre_array(i), n);
			Rgrp = aa2mat(ang_grp + ang_grp_incre_array(i), n);

			updatePlot(Robj, Rgrp, gp10_w, gp20_w, com_w, points_w, dir, h_vertices, h_cp, h_com, h_gravity, h_fingertip, h_gripper, h_surf);
		end
		ang_obj = ang_obj + ang_obj_incre;
		ang_grp = ang_grp + ang_grp_incre;

	end

	% update states
	[gp10_w, gp20_w, com_w, points_w, dir] = updateStates(gp10_w, gp20_w, com_w, points_w, dir, Robj, Rgrp);
    q_now = quatMTimes(mat2quat(Robj), q_now);
	% --------------------------------------
	% 	Post-Rolling
    %   From q_now to qf
	% --------------------------------------
	num = floor(angBTquat(q_now, qf)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(q_now, qf, t);

	h_fingertip.Color      = [0.9 0.1 0];
	h_fingertip.MarkerSize = 40;

	for s = 1:length(t)
		q0i_w = quatMTimes(qi(:,s), quatInv(q_now)); % a rotation that rotates q_now to qi(:,s), measured in world frame
		Robj  = quat2m(q0i_w);
		Rgrp  = Robj;
		updatePlot(Robj, Rgrp, gp10_w, gp20_w, com_w, points_w, dir, h_vertices, h_cp, h_com, h_gravity, h_fingertip, h_gripper, h_surf);
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



function updatePlot(Robj, Rgrp, gp1, gp2, com, points, dir, h_vertices, h_cp, h_com, h_gravity, h_fingertip, h_gripper, h_surf)
	gp1_ = Robj*gp1;
	gp2_ = Robj*gp2;
	com_ = Robj*com;
	ps_  = Robj*points;
	dir_ = Rgrp*dir;

	gbase1_ = gp1_ + 2*dir_;
	gbase2_ = gp2_ + 2*dir_;

	[~, cpid] = min(ps_(3,:));
	cpid      = find(abs(ps_(3,:) - ps_(3,cpid))<1e-5);
	cp_       = ps_(:,cpid);

	% draw
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
	h_fingertip.XData = [gp1_(1) gp2_(1)]; 
	h_fingertip.YData = [gp1_(2) gp2_(2)]; 
	h_fingertip.ZData = [gp1_(3) gp2_(3)];
	h_gripper.XData   = [gp1_(1) gbase1_(1) gbase2_(1) gp2_(1)]; 
	h_gripper.YData   = [gp1_(2) gbase1_(2) gbase2_(2) gp2_(2)]; 
	h_gripper.ZData   = [gp1_(3) gbase1_(3) gbase2_(3) gp2_(3)];
	h_surf.Vertices   = ps_';


	axis([com_(1)-1 com_(1)+1 com_(2)-1 com_(2)+1 com_(3)-1 com_(3)+1]);

	drawnow;
end

function [gp1o_w, gp2o_w, com, points, dir] = updateStates(gp1o_w, gp2o_w, com, points, dir, Robj, Rgrp)
	% update states
	gp1o_w = Robj*gp1o_w;
	gp2o_w = Robj*gp2o_w;
	com    = Robj*com;
	points = Robj*points;
	dir    = Rgrp*dir;
end