function handles = plotObject(mesh, fidOrhandle, q, handles)
if nargin <= 2
	q = [1 0 0 0]';
end

if nargin <= 1
	fidOrhandle = 1;
end

m = quat2m(q);
vertices = m*(mesh.vertices');

if ~isempty(fidOrhandle)
	if isnumeric(fidOrhandle)
		figure(fidOrhandle);
	    clf;
	else
		axes(fidOrhandle);
	    cla;
	end
end

hold on;
if nargin >= 4
	handles.Vertices   = vertices';
else
	handles = patch('Faces', mesh.faces, ...
				  'Vertices', vertices', ...
				  'FaceColor',       [0.8 0.8 1.0], ...
			      'EdgeColor',       'none',        ...
			      'FaceLighting',    'gouraud',     ...
			      'AmbientStrength', 0.15);
end

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('SHINY');
% material('dull');


xlabel('X'); ylabel('Y'); zlabel('Z');

axis equal;
view(-43, 27);

end