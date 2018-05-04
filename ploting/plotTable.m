function plotTable(fidOrhandle)
if nargin == 0
	fidOrhandle = 1;
end

if isnumeric(fidOrhandle)
	figure(fidOrhandle);
else
	axes(fidOrhandle);
end

% table
table.vertices      = zeros(3,8);
table.vertices(:,1) = [-59 255 250]';
table.vertices(:,2) = [241  255 250]';
table.vertices(:,3) = [241  255 0]';
table.vertices(:,4) = [-59 255 0]';
table.vertices(:,5) = [241   555 250]';
table.vertices(:,6) = [-59  555 250]';
table.vertices(:,7) = [-59  555 0]';
table.vertices(:,8) = [241   555 0]';
table.vertices      = (table.vertices)';
table.faces = [1 4 3;
	   1 2 3;
	   6 1 2;
	   6 2 5;
	   6 8 5;
	   6 8 7;
	   3 7 4;
	   3 7 8;
	   1 7 6;
	   1 7 4;
	   2 8 5;
	   2 8 3];
patch('Faces', table.faces, ...
				  'Vertices', table.vertices, ...
				  'FaceColor',       [0.6 0.8 0.4], ...
			      'EdgeColor',       'none',        ...
			      'FaceLighting',    'gouraud',     ...
			      'AmbientStrength', 0.15);

xlabel('X'); ylabel('Y'); zlabel('Z');
axis([-59 241 255 555 100 500]);
view(166, 22);
axis off