% object def
% 	mlist: list of modes
%	plist: list of vertices of mesh model
function [fgraph, pgraph, mesh, mesh_s] = getObject(para, stlname) 

disp('[GetObject] Reading STL model..');
fv          = stlread(stlname);
[v, f]      = patchslim(fv.vertices, fv.faces);
fv.vertices = v;
fv.faces    = f;

disp('[GetObject] Processing Model..');
% --------------------------------------------
% 		geometry
% --------------------------------------------
% scaling
L = max([max(fv.vertices(:, 1)) - min(fv.vertices(:, 1)), 
		 max(fv.vertices(:, 2)) - min(fv.vertices(:, 2)), 
		 max(fv.vertices(:, 3)) - min(fv.vertices(:, 3))]);
fv.vertices = fv.vertices/L;

points = fv.vertices';
faces  = fv.faces';
NP     = size(points, 2);
% NF     = size(faces, 2);
disp(['[GetObject] Points on Model: ' num2str(NP) ]);

% calculate center of mass
COM = mean(points, 2); % an estimation

% calculate the area of each triangle
A = points(:, faces(1,:));
B = points(:, faces(2,:));
C = points(:, faces(3,:));
a = normByCol(A-B);
b = normByCol(B-C);
c = normByCol(C-A);
p = (a+b+c)/2;
area = sqrt(p.*(p-a).*(p-b).*(p-c));

% --------------------------------------------
% 		Decimated Mesh
% --------------------------------------------
if size(fv.faces, 1) > 1000
	mesh_s           = reducepatch(fv.faces, fv.vertices, 500);
	mesh_s.err_bound = hausdorffDist(mesh_s.vertices', fv.vertices');
else
	mesh_s           = fv;
	mesh_s.err_bound = 0;
end
disp(['[GetObject] Mesh Approximation Error: ' num2str(mesh_s.err_bound) ]);
% assert(mesh_s.err_bound < 1e-2, 'Approximation Error is too large.');

% --------------------------------------------
% 		Convex Hull
% 		And Decimation
% --------------------------------------------
fvc_id    = convhull(fv.vertices, 'simplify',true);
fvr       = reducepatch(fvc_id, fv.vertices, 100);
err_bound = hausdorffDist(fv.vertices(fvc_id, :)', fvr.vertices');
NFC       = size(fvr.faces, 1);
NPC       = size(fvr.vertices, 1);
assert(NPC == max(max(fvr.faces)));
disp(['[GetObject] Convex Hull Approximation Error: ' num2str(err_bound) ]);

% face_conv = convhull(fv.vertices, 'simplify',true);
% NFC       = size(face_conv, 1); 
% ch_pid    = unique(face_conv);
% NPC		  = length(ch_pid);
% ch_pid_ch = 1:NPC;
% [~,idx] = ismember(face_conv, ch_pid);
% face_conv = ch_pid_ch(idx);

% get adjacent matrix
pch_adj_matrix = sparse( fvr.faces(:,1), fvr.faces(:,2), 1, NPC, NPC ) + ...
			     sparse( fvr.faces(:,2), fvr.faces(:,3), 1, NPC, NPC ) + ...
			     sparse( fvr.faces(:,3), fvr.faces(:,1), 1, NPC, NPC );
pch_adj_matrix = (pch_adj_matrix + pch_adj_matrix.')>0;

disp(['[GetObject] Points on Convex Hull: ' num2str(length(unique(fvc_id))) ]);
disp(['[GetObject] Points on Simplified Convex Hull: ' num2str(NPC) ]);

% --------------------------------------------
% 		stable modes
% --------------------------------------------
% find stable modes
m_is_stable    = false(1, NFC);
m_quat         = zeros(4, NFC); % quaternion to transform Normal to [0 0 1]
for m = 1:NFC
	a = fvr.vertices(fvr.faces(m, 1), :)';
	b = fvr.vertices(fvr.faces(m, 2), :)';
	c = fvr.vertices(fvr.faces(m, 3), :)';

	[~, in] = projectOntoTri(a, b, c, COM);

	if in
		m_is_stable(m)   = true;
		
		% get normal
	    b = cross(a - b, c - b);
	    if norm(b) < 1e-7
	        error('Degenerated tri-angle');
	    end
	    b = b/norm(b);
	    if b'*(COM - a) < 0
		    % b should point inward
	    	b = -b;
	    end
		q           = quatBTVec(b, [0 0 1]');
		m_quat(:,m) = q;
	end
end
NFS    = sum(m_is_stable);
m_quat = m_quat(:, m_is_stable); % quaternion to transform Normal to [0 0 1]
disp(['[GetObject] Stable modes: ' num2str(NFS) ]);

% --------------------------------------------
% 		Outputs
% --------------------------------------------
% stable modes
fgraph.NM   = NFS;
fgraph.quat = m_quat;

% Decimated convex hull
pgraph.NPC        = NPC;
pgraph.NFC        = NFC;
pgraph.err_bound  = err_bound;
pgraph.faces      = fvr.faces';
pgraph.vertices   = fvr.vertices';
pgraph.adj_matrix = pch_adj_matrix; 

% full mesh
mesh.faces    = faces'; % Nx3, common mesh convention in matlab
mesh.vertices = points'; 
mesh.COM      = COM;
mesh.area     = area;

disp('[GetObject] Model Processing Done.');

% --------------------------------------------
% 		plotting
% --------------------------------------------
if para.showObject
	disp('[GetObject] Plotting Object:');
	% full mesh
	plotObject(mesh, para.showObject_id(1));

	% convex hull
	figure(para.showObject_id(2)); clf; hold on;
	plot3(fv.vertices(fvc_id, 1), fv.vertices(fvc_id, 2), fv.vertices(fvc_id, 3), '.');
	patch('Faces', fvc_id, 					...
		  'vertices', fv.vertices,  		...
		  'FaceColor',       [0.8 0.8 1.0], ...
	      'EdgeColor',       'none',        ...
	      'FaceLighting',    'gouraud',     ...
	      'AmbientStrength', 0.15);
	camlight('headlight'); material('dull'); axis equal; view(-43, 27);
	% Decimated convex hull
	figure(para.showObject_id(3)); clf; hold on;
	plot3(fvr.vertices(:,1), fvr.vertices(:,2), fvr.vertices(:,3), '.');
	patch(fvr, ...
		  'FaceColor',       [0.8 0.8 1.0], ...
	      'EdgeColor',       'none',        ...
	      'FaceLighting',    'gouraud',     ...
	      'AmbientStrength', 0.15);
	camlight('headlight'); material('dull'); axis equal; view(-43, 27);
	% [dist, id1, id2] = hausdorffDist(fv.vertices(fvc_id, :)', fvr.vertices');
	% plot3([fv.vertices(fvc_id(id1), 1) fvr.vertices(id2, 1)], [fv.vertices(fvc_id(id1), 2) fvr.vertices(id2, 2)], [fv.vertices(fvc_id(id1), 3) fvr.vertices(id2, 3)], '-r.');
end

disp('[GetObject] Done.');

end