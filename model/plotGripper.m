% q: orientation of object
% gp: original position of the grasp points
% gq: orientation of gripper
function plotGripper(fidOrhandle, gripper, q, gp, gq)
if isnumeric(fidOrhandle)
	figure(fidOrhandle);
else
	axes(fidOrhandle);
end

hold on;

gp    = reshape(gp,[3 2]);
gp_q  = quatOnVec(gp, q);

% move gripper mesh
fingertip_plus  = bsxfun(@plus, quatOnVec(gripper.vertices{1}, gq), gp_q(:,1));
fingertip_minus = bsxfun(@plus, quatOnVec(gripper.vertices{2}, gq), gp_q(:,2));
finger_plus     = bsxfun(@plus, quatOnVec(gripper.vertices{3}, gq), gp_q(:,1));
finger_minus    = bsxfun(@plus, quatOnVec(gripper.vertices{4}, gq), gp_q(:,2));
palm_plus       = bsxfun(@plus, quatOnVec(gripper.vertices{5}, gq), gp_q(:,1));
palm_minus      = bsxfun(@plus, quatOnVec(gripper.vertices{6}, gq), gp_q(:,2));
palm            = [palm_plus palm_minus];

% plot
% finger tips
trisurf(gripper.faces{1}, fingertip_plus(1,:), fingertip_plus(2,:), fingertip_plus(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.9, 'linewidth', 0.1);
trisurf(gripper.faces{2}, fingertip_minus(1,:), fingertip_minus(2,:), fingertip_minus(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.9, 'linewidth', 0.1);
% finger
trisurf(gripper.faces{3}, finger_plus(1,:), finger_plus(2,:), finger_plus(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.5);
trisurf(gripper.faces{4}, finger_minus(1,:), finger_minus(2,:), finger_minus(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.5);
% palm
trisurf(gripper.faces{5}, palm(1,:), palm(2,:), palm(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.7);

% % old
% dir  = quatOnVec([0 0 1]', gq);
% base = gp_q + 1*dir*[1 1];
% plot3(gp_q(1,:), gp_q(2,:), gp_q(3,:), 'r.', 'markersize', 30); 
% plot3([gp_q(1,1) base(1,1) base(1,2) gp_q(1,2)], [gp_q(2,1) base(2,1) base(2,2) gp_q(2,2)], [gp_q(3,1) base(3,1) base(3,2) gp_q(3,2)], '-k','linewidth', 5);

axis equal;
end