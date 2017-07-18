function [] = buildMapEdges(mode_graph, mesh)
% --------------------------------------------
% 		Parameters
% --------------------------------------------
% friction between object and  ground
MU = 0.5;
cone = atan(MU);
% friction between object and finger tip
ANGLE_TOL = 0.1; % rad

% grasp pos sample density
GS_DENSITY = 0.1; % 1 point every 0.1 m^2

global LIFT_TOLERANCE GRIPPER_TILT_LIMIT GRIPPER_Z_LIMIT
LIFT_TOLERANCE     = 5*pi/180; % lift tolerance
GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
GRIPPER_Z_LIMIT    = 0.2; % finger position limit

% --------------------------------------------
% 		sample grasp positions
% --------------------------------------------
N_F          = size(mesh.faces, 2); % number of faces
grasp_points = zeros(3, N_F*50, 2);
grasp_count  = 0;

for i = 1:N_F-1
	p1 = zeros(3);
	p2 = zeros(3);
	p1(:,1) = mesh.points(:, mesh.faces(1, i));
	p1(:,2) = mesh.points(:, mesh.faces(2, i));
	p1(:,3) = mesh.points(:, mesh.faces(3, i));
	n1 = cross(p1(:,1) - p1(:,2), p1(:,3) - p1(:,2));
	area1 = norm(n1)/2;
	if area1 < 1e-7
		continue;
	end
	n1 = n1/area1/2;

	% sample grasp points on face i
	ns_i              = ceil(area1/GS_DENSITY);
	grasp_points_face_i = sampleTriUniform(p1(:,1), p1(:,2),p1(:,3), ns_i);

	for j = i+1:N_F
		p2(:,1) = mesh.points(:, mesh.faces(1, j));
		p2(:,2) = mesh.points(:, mesh.faces(2, j));
		p2(:,3) = mesh.points(:, mesh.faces(3, j));
		n2      = cross(p2(:,1) - p2(:,2), p2(:,3) - p2(:,2));
		area2   = norm(n2)/2;
		% check area 
		if area2 < 1e-7
			continue;
		end
		n2 = n2/area2/2;
		% check normal
		if acos(abs(n1'*n2)) > ANGLE_TOL
			continue;
		end

		% good, project grasp points
		[grasp_points_face_j, in] = projectOntoTri(p2(:,1),p2(:,2),p2(:,3), grasp_points_face_i);
		Nin = sum(in); 
		grasp_points(:, grasp_count+1:grasp_count+Nin, 1) = grasp_points_face_i(:, in);
		grasp_points(:, grasp_count+1:grasp_count+Nin, 2) = grasp_points_face_j(:, in);
		grasp_count = grasp_count + Nin;
	end
end

grasp_points(:, grasp_count+1:end,:) = [];

disp('Number of candidate grasps:');
disp(grasp_count);

% visualize all the grasps
figure(1); hold on;
for i = 1:grasp_count
    gp = reshape(grasp_points(:,i,:), [3,2]);
	% trisurf(mesh.faces, mesh.points(1,:), mesh.points(2,:), mesh.points(3,:), intersect*1.0, 'FaceAlpha', 0.3);
	% plot3([orig(1) dest(1)], [orig(2) dest(2)], [orig(3) dest(3)], '-o','linewidth',3 );
	plot3(gp(1,:), gp(2,:), gp(3,:), '-*','linewidth',2);
	drawnow;
end

% --------------------------------------------
% 		trim edges based on physics
% --------------------------------------------
% get list of edges
[idin, idout] = ind2sub(find(mgraph_connectmatrix));

edge_idin  = zeros(1000,1); % id of input mode
edge_idout = zeros(1000,1); % id of output mode
ga_count   = 1;

for i = 1:length(idin)
	id_good_grasp = evalGrasps(idin(i), idout(i), m_type, m_is_stable, grasp_points);

	if isempty(grasp_area0)
		mgraph_connectmatrix(idin(i), idout(i)) = 0;
	else
		grasp_area{ga_count} = grasp_area0;
		edge_idin(ga_count)  = idin(i);
		edge_idout(ga_count) = idout(i);
		ga_count             = ga_count + 1;
	end
end

mode_graph.edge.idin       = edge_idin(1:ga_count-1);
mode_graph.edge.idout      = edge_idout(1:ga_count-1);


end

function n = colnorm(a)
	n = sqrt(sum(a.^2));
end

function n = rownorm(a)
	n = sqrt(sum(a.^2,2));
end

function q = ddr(a,b)
q = bsxfun(@rdivide, a,b);
end
