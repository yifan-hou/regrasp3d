% gp: grasp points in world frame
% q: 4x1
function [] = animatePlan(fidOrhandle, plan)
global mesh gripper grasps para



% --------------------------------------
% 	draw and get the handles
% --------------------------------------
gp1o_w = grasps.points(:, plan{1}.grasp_id, 1);
gp2o_w = grasps.points(:, plan{1}.grasp_id, 2);
% object states
q0       = plan{1}.qobj(:,1);
m0_o     = quat2m(q0);
vertices = m0_o*(mesh.vertices');

z0 = min(vertices(3,:));
offset = para.scene_offset;
offset(3) = offset(3) - z0;

points_w = bsxfun(@plus, vertices, offset);
com_w    = m0_o*mesh.COM + offset;
gp1      = m0_o*gp1o_w + offset;
gp2      = m0_o*gp2o_w + offset;

if isnumeric(fidOrhandle)
	figure(fidOrhandle);
    clf;
else
	axes(fidOrhandle);
    cla;
end

% Initial plotting, get handles,
% contact points
[~, cpid] = min(points_w(3,:)); 
cpid      = abs(points_w(3,:) - points_w(3,cpid))<1e-5;
cp_w      = points_w(:,cpid);

% new plot
hold on;
% object
handles_object.surf    = plotObject(mesh, fidOrhandle, q0); % call plotObject without handle argument will clean the figure
handles_gripper        = plotGripper(fidOrhandle, gripper, q0, [gp1 gp2], plan{1}.qgrp(:,1));
handles_object.cp      = plot3(cp_w(1,:), cp_w(2,:), cp_w(3,:), '.k', 'markersize', 30);
handles_object.com     = plot3(com_w(1), com_w(2), com_w(3), 'r*', 'markersize', 8);
% handles_object.gravity = plot3(com_w(1)+[0 0], com_w(2)+[0 0], com_w(3)+[0 -0.4], 'r-', 'linewidth', 2);

plotTable(fidOrhandle);

% draw coordinate system for object
arrowLength = 1*(max(points_w(1,:)) - min(points_w(1,:)));
stemWidth   = 0.01*arrowLength;
tipWidth    = 0.015*arrowLength;

% xobj = arrowLength*quatOnVec([1 0 0]', q0);
% yobj = arrowLength*quatOnVec([0 1 0]', q0);
% zobj = arrowLength*quatOnVec([0 0 1]', q0);
% handles_object.arrow_x = mArrow3(com_w, com_w+xobj, 'color','r', 'stemWidth', stemWidth, 'tipWidth', tipWidth);
% handles_object.arrow_y = mArrow3(com_w, com_w+yobj, 'color','g', 'stemWidth', stemWidth, 'tipWidth', tipWidth);
% handles_object.arrow_z = mArrow3(com_w, com_w+zobj, 'color','b', 'stemWidth', stemWidth, 'tipWidth', tipWidth);

gp0   = (gp1 + gp2)/2;
xgrp = arrowLength*quatOnVec([1 0 0]', plan{1}.qgrp(:,1));
ygrp = arrowLength*quatOnVec([0 1 0]', plan{1}.qgrp(:,1));
zgrp = arrowLength*quatOnVec([0 0 1]', plan{1}.qgrp(:,1));
handles_gripper.arrow_x = mArrow3(gp0, gp0+xgrp, 'color','r', 'stemWidth', stemWidth, 'tipWidth', tipWidth);
handles_gripper.arrow_y = mArrow3(gp0, gp0+ygrp, 'color','g', 'stemWidth', stemWidth, 'tipWidth', tipWidth);
handles_gripper.arrow_z = mArrow3(gp0, gp0+zgrp, 'color','b', 'stemWidth', stemWidth, 'tipWidth', tipWidth);


% --------------------------------------
% 	Begin Animation
% --------------------------------------
N = length(plan);
for p = 1:N
	gp1o_w = grasps.points(:, plan{p}.grasp_id, 1);
	gp2o_w = grasps.points(:, plan{p}.grasp_id, 2);
% 	offset = [0 0 0]';
	trans  = [0 0]';

    rtype = 0;
    gp = gp0;
	for fr = 1:plan{p}.N
		qobj  = plan{p}.qobj(:, fr);
		qgrp  = plan{p}.qgrp(:, fr);
		% gp1   = plan{p}.gp1(:, fr);
		% gp2   = plan{p}.gp2(:, fr);
% 		trans = plan{p}.trans(:, fr);
		rtype = plan{p}.rtype(fr);

		if rtype
			handles_gripper.fingertip_plus.FaceColor  = [0.9 0.1 0];
			handles_gripper.fingertip_minus.FaceColor = [0.9 0.1 0];
		else
			handles_gripper.fingertip_plus.FaceColor  = [0.1 0.0 0.9];
			handles_gripper.fingertip_minus.FaceColor = [0.1 0.0 0.9];
		end

		trans = trans + plan{p}.gpxy_delta(:,fr);

		% rotate & translate
		Robj = quat2m(qobj);
		ps   = Robj*(mesh.vertices');
		gp1  = Robj*gp1o_w;
		gp2  = Robj*gp2o_w;

		gp_org = (gp1 + gp2)/2;

		z0 = min(ps(3,:));
        gp(1:2) = gp0(1:2) + trans;
        gp(3) = gp_org(3) - z0 + para.scene_offset(3);

        offset = gp - gp_org;
		% translation is described in trans,
		% so we need to get rid of xy in gp
		ps_         = ps + offset;
		com_        = Robj*mesh.COM + offset;
		gp1_        = gp1 + offset;
		gp2_        = gp2 + offset;

		cpid = ps_(3,:) < 1e-5;
		cp_  = ps_(:,cpid);

% 		% scene offset
% 		ps_         = bsxfun(@plus, ps_, para.scene_offset);
% 		com_        = com_ + para.scene_offset;
% 		gp1_        = gp1_ + para.scene_offset;
% 		gp2_        = gp2_ + para.scene_offset;

		aobj.x = arrowLength*quatOnVec([1 0 0]', qobj);
		aobj.y = arrowLength*quatOnVec([0 1 0]', qobj);
		aobj.z = arrowLength*quatOnVec([0 0 1]', qobj);
		agrp.x = arrowLength*quatOnVec([1 0 0]', qgrp);
		agrp.y = arrowLength*quatOnVec([0 1 0]', qgrp);
		agrp.z = arrowLength*quatOnVec([0 0 1]', qgrp);

		updatePlot(qobj, qgrp, gp1_, gp2_, cp_, com_, ps_, aobj, agrp, handles_object, handles_gripper);
        pause(0.1);
	end
end


end



function [] = updatePlot(qobj, qgp0, gp1, gp2, cp_, com_, ps_, aobj, agrp, handles_object, handles_gripper) 
	global gripper
	
	% update plot
	handles_object.cp.XData        = cp_(1,:);
	handles_object.cp.YData        = cp_(2,:);
	handles_object.cp.ZData        = cp_(3,:);
	handles_object.com.XData       = com_(1);
	handles_object.com.YData       = com_(2);
	handles_object.com.ZData       = com_(3);
% 	handles_object.gravity.XData   = com_(1)+[0  0]; 
% 	handles_object.gravity.YData   = com_(2)+[0  0]; 
% 	handles_object.gravity.ZData   = com_(3)+[0 -1];

	handles_object.surf.Vertices   = ps_';

	gp = (gp1+gp2)/2;

	mArrow3(gp, gp+agrp.x, 'handle', handles_gripper.arrow_x);
	mArrow3(gp, gp+agrp.y, 'handle', handles_gripper.arrow_y);
	mArrow3(gp, gp+agrp.z, 'handle', handles_gripper.arrow_z);

% 	mArrow3(com_, com_+aobj.x, 'handle', handles_object.arrow_x);
% 	mArrow3(com_, com_+aobj.y, 'handle', handles_object.arrow_y);
% 	mArrow3(com_, com_+aobj.z, 'handle', handles_object.arrow_z);

	plotGripper([], gripper, qobj, [gp1 gp2], qgp0, handles_gripper);
	


	% axis([com_(1)-1 com_(1)+1 com_(2)-1 com_(2)+1 com_(3)-1 com_(3)+1]);
	% axis equal;
    axis off
	drawnow;
end
