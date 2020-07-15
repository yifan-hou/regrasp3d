function handles = plotObject(mesh, fidOrhandle, q, handles)
% global para

if nargin <= 2
	q = [1 0 0 0]';
end

if nargin <= 1
	fidOrhandle = 1;
end

m = quat2SO3(q);
% T = para.scene_offset;

vertices      = m*(mesh.vertices');
minz          = min(vertices(3,:));
vertices(3,:) = vertices(3,:) - minz;
% vertices      = bsxfun(@plus, vertices, T);

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
view(166, 22);

end