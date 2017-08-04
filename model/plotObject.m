function plotObject(mesh, fidOrhandle, q)
if nargin <= 2
	q = [1 0 0 0]';
end

if nargin <= 1
	fidOrhandle = 1;
end

for i = 1:size(mesh.points,2)
	mesh.points(:,i) = quatOnVec(mesh.points(:,i), q);
end

if isnumeric(fidOrhandle)
	figure(fidOrhandle);
    clf;
else
	axes(fidOrhandle);
    cla;
end

hold on;
plot3(mesh.points(1,:), mesh.points(2,:), mesh.points(3,:), '.');
% trisurf(mesh.faces',mesh.points(1,:), mesh.points(2,:), mesh.points(3,:), 'Facecolor', 'b','FaceAlpha', 0.3);
patch('Faces', mesh.faces', ...
	  'Vertices', mesh.points', ...
	  'FaceColor',       [0.8 0.8 1.0], ...
      'EdgeColor',       'none',        ...
      'FaceLighting',    'gouraud',     ...
      'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');


xlabel('X'); ylabel('Y'); zlabel('Z');

axis equal;
view(-43, 27);

end