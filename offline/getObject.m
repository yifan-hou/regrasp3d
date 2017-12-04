% object def
% 	mlist: list of modes
%	plist: list of vertices of mesh model
function [fgraph, pgraph, mesh, mesh_s] = getObject(para, COM, stlname) 

disp('[GetObject] Reading STL model..');
fv          = stlread(stlname);
[v, f]      = patchslim(fv.vertices, fv.faces);
fv.vertices = v;
fv.faces    = f;

disp('[GetObject] Processing Model..');
% --------------------------------------------
% 		geometry
% --------------------------------------------
% % scaling
% L = max([max(fv.vertices(:, 1)) - min(fv.vertices(:, 1)), 
% 		 max(fv.vertices(:, 2)) - min(fv.vertices(:, 2)), 
% 		 max(fv.vertices(:, 3)) - min(fv.vertices(:, 3))]);
% fv.vertices = fv.vertices/L;

points = fv.vertices';
faces  = fv.faces';
NP     = size(points, 2);
% NF     = size(faces, 2);
disp(['[GetObject] Points on Model: ' num2str(NP) ]);

% % calculate center of mass
% COM = mean(points, 2); % an estimation

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
points_s = fv.vertices';
faces_s  = fv.faces';

mesh_s.COM      = COM;

% calculate the area of each triangle
A = points_s(:, faces_s(1,:));
B = points_s(:, faces_s(2,:));
C = points_s(:, faces_s(3,:));
a = normByCol(A-B);
b = normByCol(B-C);
c = normByCol(C-A);
p = (a+b+c)/2;
mesh_s.area = sqrt(p.*(p-a).*(p-b).*(p-c));

% --------------------------------------------
% 		Convex Hull
% 		And Decimation
% --------------------------------------------
fvc.faces                 = convhull(fv.vertices, 'simplify',true);
[fvc.vertices, fvc.faces] = patchslim(fv.vertices, fvc.faces);
[fvr, err_bound]          = simplifyConvHull(fvc, para);
disp(['[GetObject] Convex Hull Approximation Error: ' num2str(err_bound) ]);

NPC = size(fvr.vertices, 1);
NFC = size(fvr.faces, 1);
assert(NPC == max(max(fvr.faces)));

% get adjacent matrix
pch_adj_matrix = sparse( fvr.faces(:,1), fvr.faces(:,2), 1, NPC, NPC ) + ...
			     sparse( fvr.faces(:,2), fvr.faces(:,3), 1, NPC, NPC ) + ...
			     sparse( fvr.faces(:,3), fvr.faces(:,1), 1, NPC, NPC );
pch_adj_matrix = (pch_adj_matrix + pch_adj_matrix.')>0;

disp(['[GetObject] Reduce from: ' num2str(size(fvc.faces, 1)) 'f, ' num2str(size(fvc.vertices, 1)) 'v' ]);
disp(['[GetObject] To:          ' num2str(NFC) 'f, ' num2str(size(fvr.vertices, 1)) 'v' ]);

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
% get rid of same orientations
if NFS > 1
	m_unique = true(1, NFS);
	for i = 2:NFS
		for j = 1:i-1
			if abs(angBTquat(m_quat(:,i), m_quat(:,j))) < 1e-3
				m_unique(i) = false;
			end
		end
	end
	NFS    = sum(m_unique);
	m_quat = m_quat(:, m_unique);
end
% get rid of unstable modes
m_is_stable = true(1, NFS);
points_r = fvr.vertices';
for i = 1:NFS
	points_r_rot = quatOnVec(points_r, m_quat(:,i));
	zmin         = min(points_r_rot(3,:));
	id_contacts  = find(points_r_rot(3,:) < zmin + para.bottom_height_tol);
	spp_id       = convhull(points_r_rot(1, id_contacts), points_r_rot(2, id_contacts));
	spp          = points_r_rot(1:2, id_contacts(spp_id)); % a convex support polygon
	A            = polyarea(spp(1,:), spp(2,:));
	if A < para.minimal_support_polygon_area
		m_is_stable(i) = false;
	end
end
NFS    = sum(m_is_stable);
m_quat = m_quat(:, m_is_stable);

m_is_stable = true(1, NFS);
disp('[GetObject] Stable modes manual checking:');
for i=1:NFS
    plotObject(mesh_s, 1, m_quat(:,i));
    % Construct a questdlg with two options
    choice = questdlg('Accept this stable pose?', ...
	'Stable Pose Checking', ...
    'Yes please','No thank you','No thank you');
    % Handle response
    switch choice
        case 'Yes please'
            disp('Pose accepted.')
            m_is_stable(i) = true;
        case 'No thank you'
            disp('Pose rejected');
            m_is_stable(i) = false;
    end
end
NFS    = sum(m_is_stable);
m_quat = m_quat(:, m_is_stable);

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
% mesh.area     = area;

disp('[GetObject] Model Processing Done.');

% --------------------------------------------
% 		plotting
% --------------------------------------------
if para.showObject
	disp('[GetObject] Plotting Object:');
	% full mesh
	plotObject(mesh, para.showObject_id(1));
	title('Object Mesh')

	% convex hull
	figure(para.showObject_id(2)); clf; hold on;
	title('Convex Hull')
	plot3(fvc.vertices(:, 1), fvc.vertices(:, 2), fvc.vertices(:, 3), 'b.');
	% plot3(sampled_cvh(1, :), sampled_cvh(2, :), sampled_cvh(3, :), '.b', 'markersize', 2);
	% plot3(sampled_cvh(1, err_id1), sampled_cvh(2, err_id1), sampled_cvh(3, err_id1), '.r', 'markersize', 8);
	patch(fvc, ...
		  'FaceColor',       [0.8 0.8 1.0], ...
	      'EdgeColor',       'none',        ...
	      'FaceLighting',    'gouraud',     ...
	      'AmbientStrength', 0.15);
	camlight('headlight'); material('dull'); axis equal; view(-43, 27);
	% Decimated convex hull
	figure(para.showObject_id(3)); clf; hold on;
	title('Decimated Convex Hull')
	plot3(fvr.vertices(:,1), fvr.vertices(:,2), fvr.vertices(:,3), '.b');
	% plot3(sampled_cvh_r(1, :), sampled_cvh_r(2, :), sampled_cvh_r(3, :), '.b', 'markersize', 2);
	% plot3(sampled_cvh_r(1, err_id2), sampled_cvh_r(2, err_id2), sampled_cvh_r(3, err_id2), '.r', 'markersize', 8);
	patch(fvr, ...
		  'FaceColor',       [0.8 0.8 1.0], ...
	      'EdgeColor',       'none',        ...
	      'FaceLighting',    'gouraud',     ...
	      'AmbientStrength', 0.15);
	camlight('headlight'); material('dull'); axis equal; view(-43, 27);
end

disp('[GetObject] Done.');

end




function [fvr, err_bound] = simplifyConvHull(fvc, para)

if size(fvc.faces, 1) < para.NF_CVR
	fvr.faces    = fvc.faces;
	fvr.vertices = fvc.vertices;
	err_bound    = 0;
	return;
end

fvr                   = reducepatch(fvc.faces, fvc.vertices, para.NF_CVR);
[err_bound, id1, id2] = hausdorffDist(fvc.vertices', fvr.vertices', true); % single side hausdorff distance
additional_points     = [];
while err_bound > para.err_tol_CVR
	% add the farest point to the reduced convex hull
	additional_points     = [additional_points; fvc.vertices(id1, :)];
	new_vertices          = [fvr.vertices; additional_points];
	[err_bound, id1, id2] = hausdorffDist(fvc.vertices', new_vertices', true); % single side hausdorff distance
end

fvr.faces                 = convhull(new_vertices, 'simplify',true);
[fvr.vertices, fvr.faces] = patchslim(new_vertices, fvr.faces);


% evaluate decimation error
[err_bound, id1, id2] = hausdorffDist(fvc.vertices', fvr.vertices');

end