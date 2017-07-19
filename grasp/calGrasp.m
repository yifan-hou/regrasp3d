function [grasps, graph] = calGrasp(graph, mesh, para)

% --------------------------------------------
% 		Parameters
% --------------------------------------------
GRIPPER_TILT_LIMIT = para.GRIPPER_TILT_LIMIT;
GRIPPER_Z_LIMIT    = para.GRIPPER_Z_LIMIT;
MU                 = para.MU;
GS_DENSITY         = para.GS_DENSITY;
ANGLE_TOL          = para.ANGLE_TOL;
verbosity          = para.verbosity;

cone = atan(MU);

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

% calculate quaternions for each grasp
grasp_quats = zeros(4, grasp_count);
for i = 1:grasp_count
	v = grasp_points(:,i,1) - grasp_points(:,i,2);
	grasp_quats(:,i) = quatBTVec(v,[1 0 0]');
end

% % visualize all the grasps
% if verbosity >= 1
% 	figure(1); hold on;
% 	for i = 1:grasp_count
% 	    gp = reshape(grasp_points(:,i,:), [3,2]);
% 		% trisurf(mesh.faces, mesh.points(1,:), mesh.points(2,:), mesh.points(3,:), intersect*1.0, 'FaceAlpha', 0.3);
% 		% plot3([orig(1) dest(1)], [orig(2) dest(2)], [orig(3) dest(3)], '-o','linewidth',3 );
% 		plot3(gp(1,:), gp(2,:), gp(3,:), '-*','linewidth',2);
% 		drawnow;
% 	end
% end

% --------------------------------------------
% 		Calculate collision free rotation range
% 		for each grasp
% --------------------------------------------
grasp_range = cell(1, grasp_count);
for i = 1:grasp_count
	% rotate all the points

	for j = 1:360
		deg = j*pi/180 - pi;
		% collide = gripperCollisionCheck(p)
	end
	grasp_range{i} = [-pi, pi];
end


grasps.count  = grasp_count;
grasps.points = grasp_points;
grasps.quats  = grasp_quats;
grasps.range  = grasp_range;



% --------------------------------------------
% 		Calculate feasible grasps 
% 		for each node
% --------------------------------------------
m_grasps = ones(graph.NM, grasp_count);
for i = 1:graph.NM
	if para.verbosity >= 2
		plotObject(mesh, 2, graph.quat(:,i));
    end
    
    % find the bottom point, vertical offset
    points_rot = mesh.points; % get dim
    for j = 1:size(mesh.points,2)
    	points_rot(:,j) = quatOnVec(mesh.points(:,j), graph.quat(:,i));
    end
    z_offset = min(points_rot(3,:));
    
	tilt_z_limit = sin(GRIPPER_TILT_LIMIT);
	for j = 1:grasp_count
		% rotate grasp
		p1 = quatOnVec(grasp_points(:,j,1), graph.quat(:, i));
		p2 = quatOnVec(grasp_points(:,j,2), graph.quat(:, i));
		if (p1(3) - z_offset < GRIPPER_Z_LIMIT) || (p2(3) - z_offset< GRIPPER_Z_LIMIT) 
			m_grasps(i, j) = 0;
			continue;
		end

		b = p1 - p2; b = b/norm(b);
		if abs(b(3)) > tilt_z_limit
			m_grasps(i, j) = 0;
			continue;
		end

		% visualization
		if para.verbosity >= 2
			plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], '*','markersize',8);
		end
	end


end

end


% 
% 	utilities
% 


function n = colnorm(a)
	n = sqrt(sum(a.^2));
end

function n = rownorm(a)
	n = sqrt(sum(a.^2,2));
end

function q = ddr(a,b)
q = bsxfun(@rdivide, a,b);
end
