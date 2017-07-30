% q: orientation of object
% gp: original position of the grasp points
% gq: orientation of gripper
function plotGripper(fid, q, gp, gq)
figure(fid); hold on;

gp = reshape(gp,[3 2]);
gp_q = gp; % get dimension
gp_q(:,1) = quatOnVec(gp(:,1), q);
gp_q(:,2) = quatOnVec(gp(:,2), q);

dir = quatOnVec([0 0 1]', gq);
base = gp_q + 2*dir*[1 1];

% fingertip
plot3(gp_q(1,:), gp_q(2,:), gp_q(3,:), 'r.', 'markersize',40); 
% finger
plot3([gp_q(1,1) base(1,1) base(1,2) gp_q(1,2)], [gp_q(2,1) base(2,1) base(2,2) gp_q(2,2)], [gp_q(3,1) base(3,1) base(3,2) gp_q(3,2)], '-k','linewidth',8);

end