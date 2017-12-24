% gp: grasp points in world frame
% q: 4x1
function animatePlan(fidOrhandle, plan)
global mesh gripper grasps



% --------------------------------------
% 	draw and get the handles
% --------------------------------------
gp1o_w = grasps.points(:, plan{1}.grasp_id, 1);
gp2o_w = grasps.points(:, plan{1}.grasp_id, 2);
% object states
q0       = plan{1}.qobj(:,1);
m0_o     = quat2m(q0);
points_w = m0_o*(mesh.vertices');
% offset   = min(points_w(3,:));
com_w    = m0_o*mesh.COM;
gp1      = m0_o*gp1o_w;
gp2      = m0_o*gp2o_w;


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
handles_object.gravity = plot3(com_w(1)+[0 0], com_w(2)+[0 0], com_w(3)+[0 -0.4], 'r-', 'linewidth', 2);

% table
table.vertices      = zeros(3,8);
table.vertices(:,1) = [-1 -1 0]';
table.vertices(:,2) = [1  -1 0]';
table.vertices(:,3) = [1  -1 -0.1]';
table.vertices(:,4) = [-1 -1 -0.1]';
table.vertices(:,5) = [1   1 0]';
table.vertices(:,6) = [-1  1 0]';
table.vertices(:,7) = [-1  1 -0.1]';
table.vertices(:,8) = [1   1 -0.1]';
table.vertices = 100*table.vertices';
table.faces = [1 4 3;
	   1 2 3;
	   6 1 2;
	   6 2 5;
	   6 8 5;
	   6 8 7;
	   3 7 4;
	   3 7 8;
	   1 7 6;
	   1 7 4;
	   2 8 5;
	   2 8 3];
patch('Faces', table.faces, ...
				  'Vertices', table.vertices, ...
				  'FaceColor',       [0.6 0.8 0.4], ...
			      'EdgeColor',       'none',        ...
			      'FaceLighting',    'gouraud',     ...
			      'AmbientStrength', 0.15);

xlabel('X'); ylabel('Y'); zlabel('Z');
axis(100*[-1 1 -1 1 0 2]);
view(-43, 27);
axis off

% --------------------------------------
% 	Begin Animation
% --------------------------------------
N = length(plan);
for p = 1:N
	gp1o_w = grasps.points(:, plan{p}.grasp_id, 1);
	gp2o_w = grasps.points(:, plan{p}.grasp_id, 2);
% 	offset = [0 0 0]';
	trans  = [0 0]';

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
		gp   = (gp1 + gp2)/2;

		offset      = [gp(1) gp(2) min(ps(3,:))]';
		offset(1:2) = offset(1:2) - trans;
		ps_         = ps - offset;
		com_        = Robj*mesh.COM - offset;
		gp1_        = gp1 - offset;
		gp2_        = gp2 - offset;

		cpid = ps_(3,:) < 1e-5;
		cp_  = ps_(:,cpid);


		updatePlot(qobj, qgrp, gp1_, gp2_, cp_, com_, ps_, handles_object, handles_gripper);
        pause(0.1);
	end
end


end



function updatePlot(qobj, qgp0, gp1, gp2, cp_, com_, ps_, handles_object, handles_gripper) 
	global mesh gripper
	
	% update plot

	handles_object.cp.XData        = cp_(1,:);
	handles_object.cp.YData        = cp_(2,:);
	handles_object.cp.ZData        = cp_(3,:);
	handles_object.com.XData       = com_(1);
	handles_object.com.YData       = com_(2);
	handles_object.com.ZData       = com_(3);
	handles_object.gravity.XData   = com_(1)+[0  0]; 
	handles_object.gravity.YData   = com_(2)+[0  0]; 
	handles_object.gravity.ZData   = com_(3)+[0 -1];

	handles_object.surf.Vertices   = ps_';

	plotGripper([], gripper, qobj, [gp1 gp2], qgp0, handles_gripper);
	

	% axis([com_(1)-1 com_(1)+1 com_(2)-1 com_(2)+1 com_(3)-1 com_(3)+1]);
	% axis equal;
    axis off
	drawnow;
end
