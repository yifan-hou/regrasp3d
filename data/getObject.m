% object def
% 	mlist: list of modes
%	plist: list of vertices of mesh model
function [fgraph pgraph mesh] = getObject(para) 

% --------------------------------------------
% 		geometry
% --------------------------------------------

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

p_connectmatrix = zeros(NP);
p_connectmatrix(1, [2 4 6]) = 1;
p_connectmatrix(2, [1 3 5]) = 1;
p_connectmatrix(3, [2 4 8]) = 1;
p_connectmatrix(4, [1 3 7]) = 1;
p_connectmatrix(5, [2 6 8]) = 1;
p_connectmatrix(6, [1 5 7]) = 1;
p_connectmatrix(7, [4 6 8]) = 1;
p_connectmatrix(8, [3 5 7]) = 1;

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

% --------------------------------------------
% 		face graph 
% --------------------------------------------
NM              = 6;
m_contact_area  = cell(NM, 1); % list of vertices for each mode
m_connectmatrix = zeros(NM); % edge matrix of mode graph
m_normal        = zeros(3, NM); % list of face normals
m_center        = zeros(3, NM); % list of face centers
m_quat          = zeros(4, NM); % quaternion to transform Normal to [0 0 1]

m_contact_area{1} = [points(:,1) points(:,2) points(:,3) points(:,4)];
m_contact_area{2} = [points(:,2) points(:,5) points(:,8) points(:,3)];
m_contact_area{3} = [points(:,5) points(:,6) points(:,7) points(:,8)];
m_contact_area{4} = [points(:,6) points(:,1) points(:,4) points(:,7)];
m_contact_area{5} = [points(:,1) points(:,6) points(:,5) points(:,2)];
m_contact_area{6} = [points(:,3) points(:,8) points(:,7) points(:,4)];

m_connectmatrix(1, [4 5 2 6]) = 1;
m_connectmatrix(2, [5 3 6 1]) = 1;
m_connectmatrix(3, [2 5 4 6]) = 1;
m_connectmatrix(4, [1 6 3 5]) = 1;
m_connectmatrix(5, [4 3 2 1]) = 1;
m_connectmatrix(6, [4 1 3 2]) = 1;

for i = 1:NM
	coordinates = m_contact_area{i};
    
    b = cross(coordinates(:,1) - coordinates(:,2), coordinates(:,3) - coordinates(:,2));
    if norm(b) < 1e-7
        error('Degenerated tri-angle');
    end
    b = b/norm(b);
    % b should point inward
    if b'*(COM - m_contact_area{i}(:,1)) < 0
    	b = -b;
    end

	q = quatBTVec(b, [0 0 1]');
    
	m_normal(:,i) = b;
	m_quat(:,i)   = q;
	m_center(:,i) = mean(coordinates,2);
end

% % pmodes: what modes does a point belong to
% pmodes    = cell(NP,1);
% pmodes{1} = [v1 e9 e4 e1  f1 f5 f4];
% pmodes{2} = [v2 e9 e7 e8  f1 f2 f5];
% pmodes{3} = [v3 e7 e6 e10 f1 f2 f6];
% pmodes{4} = [v4 e1 e2 e10 f1 f4 f6];
% pmodes{5} = [v5 e5 e8 e12 f2 f3 f5];
% pmodes{6} = [v6 e3 e4 e12 f3 f4 f5];
% pmodes{7} = [v7 e2 e3 e11 f3 f4 f6];
% pmodes{8} = [v8 e5 e6 e11 f2 f3 f6];

fgraph.NM            = NM;
fgraph.contact_area  = m_contact_area;
fgraph.normal        = m_normal;
fgraph.center        = m_center;
fgraph.connectmatrix = m_connectmatrix;
fgraph.quat          = m_quat;


pgraph.NP            = NP;
pgraph.points        = points;
pgraph.connectmatrix = p_connectmatrix;



mesh.faces      = faces;
mesh.points     = points;
mesh.COM        = COM;
% mesh.pmodes     = pmodes;

% --------------------------------------------
% 		plotting
% --------------------------------------------
if para.showObject
	plotObject(mesh, para.showObject_id);
	for i = 1:NM
		plot3(m_center(1,i), m_center(2,i), m_center(3,i),'.','markersize',15);
		quiver3(m_center(1,i), m_center(2,i), m_center(3,i), m_normal(1,i), m_normal(2,i), m_normal(3,i));
	end
end

end