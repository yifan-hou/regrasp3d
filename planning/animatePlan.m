% gp: grasp points in world frame
% q: 4x1
function animatePlan(mesh, grasps, gripper, fidOrhandle, path_q, path_graspid, path_qp, plan_2d)

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
gp1o_w    = grasps.points(:,path_graspid(1), 1);
gp2o_w    = grasps.points(:,path_graspid(1), 2);
gp10_w    = quatOnVec(gp1o_w, q0);
gp20_w    = quatOnVec(gp2o_w, q0);
qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate

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
handles_gripper        = plotGripper(fidOrhandle, gripper, q0, [gp1o_w gp2o_w], qgrasp0_w);
handles_object.cp      = plot3(cp_w(1,:), cp_w(2,:), cp_w(3,:), '.k', 'markersize', 30);
handles_object.com     = plot3(com_w(1), com_w(2), com_w(3), 'r*', 'markersize', 8);
handles_object.gravity = plot3(com_w(1)+[0 0], com_w(2)+[0 0], com_w(3)+[0 -0.4], 'r-', 'linewidth', 2);

xlabel('X'); ylabel('Y'); zlabel('Z');
view(-43, 27);

NP = size(path_q, 2);
for p = 1:NP-1
	q0       = path_q(:,p);
	qf       = path_q(:,p+1);
	qp       = path_qp(:,p);

	% grasp
    gp1o_w    = grasps.points(:,path_graspid(p), 1);
	gp2o_w    = grasps.points(:,path_graspid(p), 2);
	gp10_w    = quatOnVec(gp1o_w, q0);
	gp20_w    = quatOnVec(gp2o_w, q0);
	qgrasp0_w = getProperGrasp(gp10_w, gp20_w); % grasp frame for q0, under world coordinate

	qobj = q0;
	qgrp = qgrasp0_w;
	pause(0.5);
	% First plot for one transition.
	% Update position of gripper
	
	updatePlot(qobj, qgrp, gp1o_w, gp2o_w, gripper, mesh.COM, mesh.vertices', handles_object, handles_gripper) 

	% --------------------------------------
	% 	Pre-Rolling to qp
	% --------------------------------------
	num = floor(angBTquat(q0, qp)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(q0, qp, t);

	handles_gripper.fingertip_plus.FaceColor  = [0.9 0.1 0];
	handles_gripper.fingertip_minus.FaceColor = [0.9 0.1 0];

% 	qobj_now = qobj;
	qgrp_now = qgrp;
	for s = 1:length(t)
		qobj_incre = quatMTimes(qi(:,s), quatInv(q0)); % a rotation that rotates q0 to qi(:,s), measured in world frame

		qobj  = qi(:,s);
		qgrp  = quatMTimes(qobj_incre, qgrp_now);
		updatePlot(qobj, qgrp, gp1o_w, gp2o_w, gripper, mesh.COM, mesh.vertices', handles_object, handles_gripper) 
	end

	% update states
	gp10_w = quatOnVec(gp10_w, qobj_incre);
	gp20_w = quatOnVec(gp20_w, qobj_incre);
    
	% --------------------------------------
	% 	Pivoting
	% --------------------------------------
	ang_obj = 0;
	ang_grp = 0;
	n       = gp10_w - gp20_w; n = n/norm(n); % rotation axis
    
    qobj_now = qobj;
    qgrp_now = qgrp;
	for s = 1:length(plan_2d{p}.grp_motion_diff)
		ang_obj_incre = plan_2d{p}.obj_motion_diff(s);
		ang_grp_incre = plan_2d{p}.grp_motion_diff(s);
		assert(~any(ang_obj_incre < 0)); 

		num = floor(max(ang_obj_incre, ang_grp_incre)/STEP_LENGTH_RAD);
		if num == 0
			num = 1;
		end

		ang_obj_incre_array = fixStepSample(0, ang_obj_incre, num);
		ang_grp_incre_array = fixStepSample(0, ang_grp_incre, num);

		if(plan_2d{p}.rtype(s) == 0)
			handles_gripper.fingertip_plus.FaceColor  = [0.9 0.1 0];
			handles_gripper.fingertip_minus.FaceColor = [0.9 0.1 0];
		else
			handles_gripper.fingertip_plus.FaceColor  = [0.1 0.0 0.9];
			handles_gripper.fingertip_minus.FaceColor = [0.1 0.0 0.9];
		end

		
		for i = 1:length(ang_obj_incre_array)	
			% calculate
			qobj_incre = aa2quat(ang_obj + ang_obj_incre_array(i), -n*plan_2d{p}.dir);
			qgrp_incre = aa2quat(ang_grp + ang_grp_incre_array(i), n);

			qobj = quatMTimes(qobj_incre, qobj_now);
			qgrp = quatMTimes(qgrp_incre, qgrp_now);
			updatePlot(qobj, qgrp, gp1o_w, gp2o_w, gripper, mesh.COM, mesh.vertices', handles_object, handles_gripper) 
		end
		ang_obj = ang_obj + ang_obj_incre;
		ang_grp = ang_grp + ang_grp_incre;

	end

	% --------------------------------------
	% 	Post-Rolling
    %   From qobj_now to qf
	% --------------------------------------
    qobj_now = qobj;
    qgrp_now = qgrp;
	num = floor(angBTquat(qobj_now, qf)/STEP_LENGTH_RAD);
	t   = (0:(num+1))/(num+1);
	qi  = quatSlerp(qobj_now, qf, t);

	handles_gripper.fingertip_plus.FaceColor  = [0.9 0.1 0];
	handles_gripper.fingertip_minus.FaceColor = [0.9 0.1 0];

	for s = 1:length(t)
		qobj  = qi(:,s);
		q0i_w = quatMTimes(qi(:,s), quatInv(qobj_now)); % a rotation that rotates qobj_now to qi(:,s), measured in world frame
		qgrp  = quatMTimes(q0i_w, qgrp_now);
		updatePlot(qobj, qgrp, gp1o_w, gp2o_w, gripper, mesh.COM, mesh.vertices', handles_object, handles_gripper) 
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



function updatePlot(qobj, qgp0, gp1, gp2, gripper, com, points, handles_object, handles_gripper) 
	Robj = quat2m(qobj);
	com_ = Robj*com;
	ps_  = Robj*points;

    
	[~, cpid] = min(ps_(3,:));
	cpid      = abs(ps_(3,:) - ps_(3,cpid))<1e-5;
	cp_       = ps_(:,cpid);

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
	 

	axis([com_(1)-1 com_(1)+1 com_(2)-1 com_(2)+1 com_(3)-1 com_(3)+1]);
	axis equal;
	drawnow;
end
