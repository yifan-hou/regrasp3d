% object def
% 	mlist: list of modes
%	plist: list of vertices of mesh model
function [fgraph, pgraph, mesh] = getObject(para, stlname) 

disp('[GetObject] Reading STL model..');
fv = stlread(stlname);

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

% --------------------------------------------
% 		Convex Hull
% --------------------------------------------
face_conv = convhull(fv.vertices, 'simplify',true);
NFC       = size(face_conv, 1); 
ch_pid    = unique(face_conv);
NPC		  = length(ch_pid);
ch_pid_ch = 1:NPC;

[~,idx] = ismember(face_conv, ch_pid);
face_conv = ch_pid_ch(idx);

% get adjacent matrix
pch_adj_matrix = sparse( face_conv(:,1), face_conv(:,2), 1, NPC, NPC ) + ...
			     sparse( face_conv(:,2), face_conv(:,3), 1, NPC, NPC ) + ...
			     sparse( face_conv(:,3), face_conv(:,1), 1, NPC, NPC );
pch_adj_matrix = (pch_adj_matrix + pch_adj_matrix.')>0;

disp(['[GetObject] Points on Convex Hull: ' num2str(NPC) ]);

% --------------------------------------------
% 		stable modes
% --------------------------------------------

% find stable modes
m_is_stable    = false(1, NFC);
m_quat         = zeros(4, NFC); % quaternion to transform Normal to [0 0 1]
for m = 1:NFC
	a = points(:, face_conv(m, 1));
	b = points(:, face_conv(m, 2));
	c = points(:, face_conv(m, 3));

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

% stable modes
fgraph.NM   = NFS;
fgraph.quat = m_quat;

% convex hull
pgraph.NPC		  = NPC;
pgraph.points_id  = ch_pid; % id of points on convex hull
pgraph.adj_matrix = pch_adj_matrix; % use id of convex hull points

% full mesh
mesh.faces  = faces;
mesh.points = points;
mesh.COM    = COM;

disp('[GetObject] Model Processing Done.');

% --------------------------------------------
% 		plotting
% --------------------------------------------
if para.showObject
	disp('[GetObject] Plotting Object:');
	plotObject(mesh, para.showObject_id);
	disp('[GetObject] Done.');
end

end