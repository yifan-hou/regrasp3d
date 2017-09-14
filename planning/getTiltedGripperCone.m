function gripper_cone_width = getTiltedGripperCone(gp1_w, gp2_w, original_cone_width)
[~, q_w]   = getProperGrasp(gp1_w, gp2_w); % measured in world frame
z_w       = quatOnVec([0 0 1]', q_w); % z axis of p frame, measured in w frame
tilted_ang = angBTVec([0 0 1]', z_w);
if tilted_ang > original_cone_width
    gripper_cone_width = [];
	return;
end
gripper_cone_width = tiltedConeAng(original_cone_width, tilted_ang);

end