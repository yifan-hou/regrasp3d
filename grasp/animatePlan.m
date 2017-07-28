% gp: grasp points in world frame
% q: 4x1
function animatePlan(mesh, fid, gp1, gp2, qgrasp, com, points, plan)
STEP_LENGTH_RAD = 3*pi/180;
% --------------------------------------
% 	draw and get the handles
% --------------------------------------
[~, cpid] = min(points(3,:));
cpid      = find(abs(points(3,:) - points(3,cpid))<1e-5);
cp        = points(:,cpid);

figure(fid);clf;hold on;
% object
h_vertices = plot3(points(1,:), points(2,:), points(3,:), '.');
h_surf     = trisurf(mesh.faces',points(1,:), points(2,:), points(3,:), 'FaceAlpha', 0.3);
h_cp	   = plot3(cp(1,:), cp(2,:), cp(3,:), '.k-', 'linewidth', 2, 'markersize', 30);
h_com	   = plot3(com(1), com(2), com(3), 'r*', 'markersize', 8);
h_gravity  = plot3(com(1)+[0 0], com(2)+[0 0], com(3)+[0 -1], 'r-', 'linewidth', 2);
% gripper
dir    = quatOnVec([0 0 1]', qgrasp);
gbase1 = gp1 + 2*dir;
gbase2 = gp2 + 2*dir;

% fingertip
h_fingertip = plot3([gp1(1) gp2(1)], [gp1(2) gp2(2)], [gp1(3) gp2(3)], 'r.', 'markersize',40); 
% finger
h_gripper = plot3([gp1(1) gbase1(1) gbase2(1) gp2(1)],...
				  [gp1(2) gbase1(2) gbase2(2) gp2(2)],...
				  [gp1(3) gbase1(3) gbase2(3) gp2(3)],...
				  '-k','linewidth',8);

xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;
view(-43, 27);

% --------------------------------------
% 	Do the animation
% --------------------------------------
ang_obj = 0;
ang_grp = 0;
n = gp1 - gp2; n = n/norm(n); % rotation axis
if plan.dir > 0
	n = -n;
end

for s = 1:length(plan.gripper_motion)
	ang_obj_incre = plan.object_motion(s);
	ang_grp_incre = plan.gripper_motion(s);
	assert(~any(ang_obj_incre < 0)); 

	num = floor(max(ang_obj_incre, ang_grp_incre)/STEP_LENGTH_RAD);
	if num == 0
		num = 1;
	end

	ang_obj_incre_array = fixStepSample(0, ang_obj_incre, num);
	ang_grp_incre_array = fixStepSample(0, ang_grp_incre, num);

	if(plan.rtype(s) == 0)
		h_fingertip.Color = [0.9 0.1 0];
		h_fingertip.MarkerSize = 40;
	else
		h_fingertip.Color = [0.1 0 0.9];
		h_fingertip.MarkerSize = 40;
	end

	for i = 1:length(ang_obj_incre_array)	
		% calculate
		Robj = aa2mat(ang_obj + ang_obj_incre_array(i), n);
		Rgrp = aa2mat(ang_grp + ang_grp_incre_array(i), n);

		gp1_ = Robj*gp1;
		gp2_ = Robj*gp2;
		com_ = Robj*com;
		ps_  = Robj*points;
		dir_ = Rgrp*dir;

		gbase1_ = gp1_ + 2*dir_;
		gbase2_ = gp2_ + 2*dir_;

		[~, cpid] = min(ps_(3,:));
		cpid = find(abs(ps_(3,:) - ps_(3,cpid))<1e-5);
		cp_ = ps_(:,cpid);

		% draw
		h_vertices.XData = ps_(1,:); h_vertices.YData = ps_(2,:); h_vertices.ZData = ps_(3,:);
		h_cp.XData = cp_(1,:); h_cp.YData = cp_(2,:); h_cp.ZData = cp_(3,:);
		h_com.XData = com_(1); h_com.YData = com_(2); h_com.ZData = com_(3);
		h_gravity.XData = com_(1)+[0 0]; h_gravity.YData = com_(2)+[0 0]; h_gravity.ZData = com_(3)+[0 -1];
		h_fingertip.XData = [gp1_(1) gp2_(1)]; h_fingertip.YData = [gp1_(2) gp2_(2)]; h_fingertip.ZData = [gp1_(3) gp2_(3)];
		h_gripper.XData = [gp1_(1) gbase1_(1) gbase2_(1) gp2_(1)]; h_gripper.YData = [gp1_(2) gbase1_(2) gbase2_(2) gp2_(2)]; h_gripper.ZData = [gp1_(3) gbase1_(3) gbase2_(3) gp2_(3)];
		h_surf.Vertices = ps_';


		drawnow;
	end
	ang_obj = ang_obj + ang_obj_incre;
	ang_grp = ang_grp + ang_grp_incre;

end

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



