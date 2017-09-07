% q: orientation of object
% gp: original position of the grasp points
% gq: orientation of gripper
function handles = plotGripper(fidOrhandle, gripper, q, gp, gq, handles)

if ~isempty(fidOrhandle)
	if isnumeric(fidOrhandle)
		figure(fidOrhandle);
	else
		axes(fidOrhandle);
	end
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
if nargin >= 6
	handles.fingertip_plus.Vertices  = fingertip_plus';
	handles.fingertip_minus.Vertices = fingertip_minus';
	handles.finger_plus.Vertices     = finger_plus';
	handles.finger_minus.Vertices    = finger_minus';
	handles.palm.Vertices            = palm';
else
	% handles.fingertip_plus  = trisurf(gripper.faces{1}, fingertip_plus(1,:), fingertip_plus(2,:), fingertip_plus(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.9, 'linewidth', 0.1);
	% handles.fingertip_minus = trisurf(gripper.faces{2}, fingertip_minus(1,:), fingertip_minus(2,:), fingertip_minus(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.9, 'linewidth', 0.1);
	% handles.finger_plus     = trisurf(gripper.faces{3}, finger_plus(1,:), finger_plus(2,:), finger_plus(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.5);
	% handles.finger_minus    = trisurf(gripper.faces{4}, finger_minus(1,:), finger_minus(2,:), finger_minus(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.5);
	% handles.palm            = trisurf(gripper.faces{5}, palm(1,:), palm(2,:), palm(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.7);
	handles.fingertip_plus  = patch('Faces', gripper.faces{1}, 'Vertices', fingertip_plus',  'FaceColor', 'k', 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
	handles.fingertip_minus = patch('Faces', gripper.faces{2}, 'Vertices', fingertip_minus', 'FaceColor', 'k', 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
	handles.finger_plus     = patch('Faces', gripper.faces{3}, 'Vertices', finger_plus',     'FaceColor', [0.3 0.3 0.6], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
	handles.finger_minus    = patch('Faces', gripper.faces{4}, 'Vertices', finger_minus',    'FaceColor', [0.3 0.3 0.6], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
	handles.palm            = patch('Faces', gripper.faces{5}, 'Vertices', palm',            'FaceColor', [0.3 0.3 0.6], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
	% Add a camera light, and tone down the specular highlighting
	% camlight('headlight');
	material('METAL');

end



axis equal;
end