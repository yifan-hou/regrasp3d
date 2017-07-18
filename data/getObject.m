% object def
% 	mlist: list of modes
%	plist: list of vertices of mesh model
function [mode_graph mesh] = getObject() 
global M_TYPE_VERTEX M_TYPE_EDGE M_TYPE_FACET
M_TYPE_VERTEX = 1;
M_TYPE_EDGE   = 2;
M_TYPE_FACET  = 3;
% --------------------------------------------
%			mode numbering
% --------------------------------------------
v1  = 1;
v2  = 2;
v3  = 3;
v4  = 4;
v5  = 5;
v6  = 6;
v7  = 7;
v8  = 8;
e1  = 9;
e2  = 10;
e3  = 11;
e4  = 12;
e5  = 13;
e6  = 14;
e7  = 15;
e8  = 16;
e9  = 17;
e10 = 18;
e11 = 19;
e12 = 20;
f1  = 21;
f2  = 22;
f3  = 23;
f4  = 24;
f5  = 25;
f6  = 26;

% --------------------------------------------
% 		get mode graph from geometry
% --------------------------------------------
NM                   = 26;
m_type               = zeros(NM, 1); % type of mode. 1:point 2:edge 3:facet
m_is_stable          = zeros(NM, 1); % is stable mode or not
m_contact_area       = cell(NM, 1); % list of vertices for each mode
mgraph_connectmatrix = zeros(NM); % edge matrix of mode graph
m_faceNormal         = cell(NM, 1); % list of face normals
m_faceCenter         = cell(NM, 1); % list of face centers

% fill in m_type, m_is_stable, m_contact_area, mgraph_connectmatrix

m_type(v1:v8)  = M_TYPE_VERTEX;
m_type(e1:e12) = M_TYPE_EDGE;
m_type(f1:f6)  = M_TYPE_FACET;

m_is_stable(f1:f6) = true;

m_contact_area{v1} = [0 0 1]';
m_contact_area{v2} = [2 0 1]';
m_contact_area{v3} = [2 0 0]';
m_contact_area{v4} = [0 0 0]';
m_contact_area{v5} = [2 3 1]';
m_contact_area{v6} = [0 3 1]';
m_contact_area{v7} = [0 3 0]';
m_contact_area{v8} = [2 3 0]';

m_contact_area{e1}  = [m_contact_area{v1} m_contact_area{v4}];
m_contact_area{e2}  = [m_contact_area{v4} m_contact_area{v7}];
m_contact_area{e3}  = [m_contact_area{v6} m_contact_area{v7}];
m_contact_area{e4}  = [m_contact_area{v1} m_contact_area{v6}];
m_contact_area{e5}  = [m_contact_area{v5} m_contact_area{v8}];
m_contact_area{e6}  = [m_contact_area{v3} m_contact_area{v8}];
m_contact_area{e7}  = [m_contact_area{v2} m_contact_area{v3}];
m_contact_area{e8}  = [m_contact_area{v2} m_contact_area{v5}];
m_contact_area{e9}  = [m_contact_area{v1} m_contact_area{v2}];
m_contact_area{e10} = [m_contact_area{v3} m_contact_area{v4}];
m_contact_area{e11} = [m_contact_area{v7} m_contact_area{v8}];
m_contact_area{e12} = [m_contact_area{v5} m_contact_area{v6}];

m_contact_area{f1} = [m_contact_area{v1} m_contact_area{v2} m_contact_area{v3} m_contact_area{v4}];
m_contact_area{f2} = [m_contact_area{v2} m_contact_area{v5} m_contact_area{v8} m_contact_area{v3}];
m_contact_area{f3} = [m_contact_area{v5} m_contact_area{v6} m_contact_area{v7} m_contact_area{v8}];
m_contact_area{f4} = [m_contact_area{v6} m_contact_area{v1} m_contact_area{v4} m_contact_area{v7}];
m_contact_area{f5} = [m_contact_area{v1} m_contact_area{v6} m_contact_area{v5} m_contact_area{v2}];
m_contact_area{f6} = [m_contact_area{v3} m_contact_area{v8} m_contact_area{v7} m_contact_area{v4}];

mgraph_connectmatrix(v1, [e9 e4 e1 f1 f5 f4])         = 1;
mgraph_connectmatrix(v2, [e9 e7 e8 f1 f2 f5])         = 1;
mgraph_connectmatrix(v3, [e7 e6 e10 f1 f2 f6])        = 1;
mgraph_connectmatrix(v4, [e1 e2 e10 f1 f4 f6])        = 1;
mgraph_connectmatrix(v5, [e5 e8 e12 f2 f3 f5])        = 1;
mgraph_connectmatrix(v6, [e3 e4 e12 f3 f4 f5])        = 1;
mgraph_connectmatrix(v7, [e2 e3 e11 f3 f4 f6])        = 1;
mgraph_connectmatrix(v8, [e5 e6 e11 f2 f3 f6])        = 1;
mgraph_connectmatrix(e1, [v1 v4 f1 f4])               = 1;
mgraph_connectmatrix(e2, [v4 v7 f4 f6])               = 1;
mgraph_connectmatrix(e3, [v6 v7 f3 f4])               = 1;
mgraph_connectmatrix(e4, [v1 v6 f4 f5])               = 1;
mgraph_connectmatrix(e5, [v5 v8 f2 f3])               = 1;
mgraph_connectmatrix(e6, [v3 v8 f2 f6])               = 1;
mgraph_connectmatrix(e7, [v2 v3 f1 f2])               = 1;
mgraph_connectmatrix(e8, [v2 v5 f2 f5])               = 1;
mgraph_connectmatrix(e9, [v1 v2 f1 f5])               = 1;
mgraph_connectmatrix(e10, [v3 v4 f1 f6])              = 1;
mgraph_connectmatrix(e11, [v7 v8 f3 f6])              = 1;
mgraph_connectmatrix(e12, [v5 v6 f3 f5])              = 1;
mgraph_connectmatrix(f1, [v1 v2 v3 v4 e1 e9 e7 e10])  = 1;
mgraph_connectmatrix(f2, [v2 v5 v8 v3 e8 e5 e6 e7])   = 1;
mgraph_connectmatrix(f3, [v5 v6 v7 v8 e5 e12 e3 e11]) = 1;
mgraph_connectmatrix(f4, [v6 v1 v4 v7 e1 e2 e3 e4])   = 1;
mgraph_connectmatrix(f5, [v1 v2 v5 v6 e4 e12 e8 e9])  = 1;
mgraph_connectmatrix(f6, [v3 v4 v7 v8 e2 e10 e11 e6]) = 1;

for i = 1:NM
	if m_type(i) ~= M_TYPE_FACET
		continue;
	end

	coordinates = m_contact_area{i};
    
    b = cross(coordinates(:,1) - coordinates(:,2), coordinates(:,3) - coordinates(:,2));
    if norm(b) < 1e-7
        error('Degenerated tri-angle');
    end
    b = b/norm(b);
    
	m_faceNormal{i} = b;
	m_faceCenter{i} = mean(coordinates,2);
end



% --------------------------------------------
% 		get point list from geometry
% --------------------------------------------
% there could be points that do not belong to any vertices
NP          = 8;
points      = zeros(3,NP);
points(:,1) = [0 0 1]';
points(:,2) = [2 0 1]';
points(:,3) = [2 0 0]';
points(:,4) = [0 0 0]';
points(:,5) = [2 3 1]';
points(:,6) = [0 3 1]';
points(:,7) = [0 3 0]';
points(:,8) = [2 3 0]';

% todo: better way to calculate COM for mesh model
COM = mean(points, 2);

faces = [1 4 3;
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
	   2 8 3]';

% check face normal directions
for i = 1:NM
	if m_type(i) ~= M_TYPE_FACET
		continue;
	end
	% b should point inward
	if m_faceNormal{i}'*(COM - m_contact_area{i}(:,1)) < 0
		m_faceNormal{i} = -m_faceNormal{i};
	end
end

% plotting
figure(1);clf;hold on;
plot3(points(1,:), points(2,:), points(3,:), '*');
text(points(1,1),points(2,1),points(3,1),  'p1');
text(points(1,2),points(2,2),points(3,2),  'p2');
text(points(1,3),points(2,3),points(3,3),  'p3');
text(points(1,4),points(2,4),points(3,4),  'p4');
text(points(1,5),points(2,5),points(3,5),  'p5');
text(points(1,6),points(2,6),points(3,6),  'p6');
text(points(1,7),points(2,7),points(3,7),  'p7');
text(points(1,8),points(2,8),points(3,8),  'p8');
trisurf(faces',points(1,:), points(2,:), points(3,:), 'FaceAlpha', 0.3);

for i = 1:NM
	if m_type(i) ~= M_TYPE_FACET
		continue;
	end
	plot3(m_faceCenter{i}(1), m_faceCenter{i}(2), m_faceCenter{i}(3),'.','markersize',15);
	quiver3(m_faceCenter{i}(1),m_faceCenter{i}(2), m_faceCenter{i}(3), m_faceNormal{i}(1), m_faceNormal{i}(2), m_faceNormal{i}(3));
end

axis equal;

% pmodes: what modes does a point belong to
pmodes    = cell(NP,1);
pmodes{1} = [v1 e9 e4 e1  f1 f5 f4];
pmodes{2} = [v2 e9 e7 e8  f1 f2 f5];
pmodes{3} = [v3 e7 e6 e10 f1 f2 f6];
pmodes{4} = [v4 e1 e2 e10 f1 f4 f6];
pmodes{5} = [v5 e5 e8 e12 f2 f3 f5];
pmodes{6} = [v6 e3 e4 e12 f3 f4 f5];
pmodes{7} = [v7 e2 e3 e11 f3 f4 f6];
pmodes{8} = [v8 e5 e6 e11 f2 f3 f6];

mode_graph.node.NM           = NM;
mode_graph.node.type         = m_type;
mode_graph.node.is_stable    = m_is_stable;
mode_graph.node.contact_area = m_contact_area;
mode_graph.node.faceNormal 	 = m_faceNormal;
mode_graph.node.faceCenter 	 = m_faceCenter;
mode_graph.connectmatrix     = mgraph_connectmatrix;

mesh.faces      = faces;

mesh.points     = points;
mesh.pmodes     = pmodes;

mesh.COM        = COM;

end