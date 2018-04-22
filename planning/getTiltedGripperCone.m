function gripper_cone_width = getTiltedGripperCone(gp1_w, gp2_w, original_cone_width)
% global grasps
% gp1o = grasps.points(:, grasp_id, 1);
% gp2o = grasps.points(:, grasp_id, 2);

% gp1_w = quatOnVec(gp1o, qnow);
% gp2_w = quatOnVec(gp2o, qnow);

a = gp1_w - gp2_w;
tilted_ang = angBTVec([0 0 1]', a);
if tilted_ang > pi/2
	tilted_ang = pi - tilted_ang;
end
tilted_ang = pi/2 - tilted_ang;
if tilted_ang > original_cone_width
    gripper_cone_width = [];
	return;
end

gripper_cone_width = tiltedConeAng(original_cone_width, tilted_ang);

end


function ang = tiltedConeAng(cone_half_ang, tilted_ang)
H = 1;
R = tan(cone_half_ang)*H;
r = tan(tilted_ang)*H;
L = sqrt(R^2 + H^2);
m = sqrt(r^2 + H^2);
theta = asin(r/R);
h = R*cos(theta);

ang = sss2aaa(h, L, m);

end
