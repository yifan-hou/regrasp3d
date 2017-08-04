% sample grasps; calculate contact mode graph
function [grasps, fgraph] = calGrasp(fgraph, pgraph, mesh, para)
disp('[CalGrasp] Sampling Grasp Positions.');
% --------------------------------------------
% 		Parameters
% --------------------------------------------
% MU                 = para.MU;
GS_DENSITY         = para.GS_DENSITY;
ANGLE_TOL          = para.ANGLE_TOL;

% cone = atan(MU);

% --------------------------------------------
% 		sample grasp positions
% --------------------------------------------
disp('[CalGrasp] Sampling Grasp Positions.');
N_F          = size(mesh.faces, 2); % number of faces
grasp_points = zeros(3, N_F*50, 2);
grasp_count  = 0;

for i = 1:N_F-1
	p1      = zeros(3);
	p2      = zeros(3);
	p1(:,1) = mesh.points(:, mesh.faces(1, i));
	p1(:,2) = mesh.points(:, mesh.faces(2, i));
	p1(:,3) = mesh.points(:, mesh.faces(3, i));
	n1      = cross(p1(:,1) - p1(:,2), p1(:,3) - p1(:,2));
	area1   = norm(n1)/2;
	if area1 < 1e-7
		continue;
	end
	n1 = n1/area1/2;

	% sample grasp points on face i
	ns_i                = ceil(area1/GS_DENSITY);
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
		
		% check distance to COM
		for k = 1:size(grasp_points_face_i, 2)
			if ~in(k)
				continue;
			end
			a = norm(mesh.COM - grasp_points_face_i(:,k));
			b = norm(mesh.COM - grasp_points_face_j(:,k));
			c = norm(grasp_points_face_i(:,k) - grasp_points_face_j(:,k));
			p = (a+b+c)/2;
			area = sqrt(p*(p-a)*(p-b)*(p-c));
			dist2COM = area*2/c;
			if dist2COM > para.COM_DIST_LIMIT
				in(k) = 0;
			end
		end

		Nin = sum(in); 
		grasp_points(:, grasp_count+1:grasp_count+Nin, 1) = grasp_points_face_i(:, in);
		grasp_points(:, grasp_count+1:grasp_count+Nin, 2) = grasp_points_face_j(:, in);
		grasp_count = grasp_count + Nin;
	end
end

grasp_points(:, grasp_count+1:end,:) = [];

disp(['[CalGrasp] Number of candidate grasps: ' num2str(grasp_count)]);

% calculate quaternions for each grasp
grasp_quats = zeros(4, grasp_count);
for i = 1:grasp_count
	v = grasp_points(:,i,1) - grasp_points(:,i,2);
	grasp_quats(:,i) = quatBTVec(v,[1 0 0]');
end

% visualize all the grasps
if para.showAllGraspSamples
	disp('[CalGrasp] Visualizing all grasps:');
	figure(para.showAllGraspSamples_id); hold on;
	for i = 1:grasp_count
	    gp = reshape(grasp_points(:,i,:), [3,2]);
		plot3(gp(1,:), gp(2,:), gp(3,:), '-*','linewidth',2);
		% drawnow;
	end
end

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
disp('[CalGrasp] Computing feasible grasps for each mode:');
m_grasps = ones(fgraph.NM, grasp_count);
for i = 1:fgraph.NM
	if para.showCheckedGrasp
		plotObject(mesh, para.showCheckedGrasp_id, fgraph.quat(:,i));
    end

    m_grasps(i,:) = checkGrasp(grasps, mesh, pgraph, fgraph.quat(:,i), para);
end

fgraph.grasps = m_grasps;

disp('[CalGrasp] Computing contact mode graph:');
% --------------------------------------------
% 		Calculate contact mode fgraph 
% --------------------------------------------
m_adj_matrix = zeros(fgraph.NM);
for i = 1:fgraph.NM
	mi = fgraph.grasps(i,:);
	for j = 1:fgraph.NM
		mj = fgraph.grasps(j,:);
		if any(mi&mj)
			m_adj_matrix(i,j) = 1;
		end
	end
end

fgraph.adj_matrix = m_adj_matrix;

disp('[CalGrasp] Done.');

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
