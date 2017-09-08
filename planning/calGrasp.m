% sample grasps; calculate contact mode graph
function [grasps, fgraph] = calGrasp(fgraph, pgraph, mesh, mesh_s, gripper, para)
% --------------------------------------------
% 		Parameters
% --------------------------------------------
% MU         = para.MU;
% GS_DENSITY = para.GS_DENSITY;
ANGLE_TOL = para.ANGLE_TOL;
NGS       = para.NGS;
% cone = atan(MU);

% --------------------------------------------
% 		sample grasp positions
% --------------------------------------------
disp('[CalGrasp] Sampling Grasp Positions.');
grasp_points         = zeros(3,   NGS,  2);
grasp_feasible_range = zeros(360, NGS);
grasp_frame          = zeros(4,   NGS);
area_bin             = mesh.area/sum(mesh.area);
grasps_count         = 1;

tic;
cctime = 0;
while true
	% 1. sample a point on the mesh surface
	samples             = mnrnd(1, area_bin);
	id_sampled          = find(samples > 0);
	p1                  = zeros(3); % the sampled triangle
	p1(:,1)             = mesh.vertices(mesh.faces(id_sampled, 1), :)';
	p1(:,2)             = mesh.vertices(mesh.faces(id_sampled, 2), :)';
	p1(:,3)             = mesh.vertices(mesh.faces(id_sampled, 3), :)';
	grasp_points_face_i = sampleTriUniform(p1(:,1), p1(:,2),p1(:,3), 1);

	% 2. check if this point belongs to a good grasp
	n1 = cross(p1(:,1) - p1(:,2), p1(:,3) - p1(:,2));
	n1 = n1/norm(n1);
	p2 = zeros(3);
	for j = 1:size(mesh.faces, 1)
		if id_sampled == j
			continue;
		end
		p2(:,1) = mesh.vertices(mesh.faces(j, 1), :)';
		p2(:,2) = mesh.vertices(mesh.faces(j, 2), :)';
		p2(:,3) = mesh.vertices(mesh.faces(j, 3), :)';
		n2      = cross(p2(:,1) - p2(:,2), p2(:,3) - p2(:,2));
		% area2   = norm(n2)/2;
		% % check area
		% if area2 < 1e-7
		% 	continue;
		% end
		% check normal
		n2 = n2/norm(n2);
		if acos(abs(n1'*n2)) > ANGLE_TOL
			continue;
		end

		% normal is good, sample the other grasp points, check angles
		Ns_j                        = floor(mesh.area(j)*para.POINTJ_SAMPLE_DENSITY)+1; % # of sample points
		grasp_points_face_j_samples = sampleTriUniform(p2(:,1), p2(:,2),p2(:,3), Ns_j);
		angle_is_good               = false;
        for s = 1:Ns_j
            % check angles 
            d12 = grasp_points_face_j_samples(:, s) - grasp_points_face_i;
            if norm(d12) < 1e-3
            	continue;
            end
            d12 = d12/norm(d12);
			if (acos(abs(n1'*d12)) > ANGLE_TOL) || (acos(abs(n2'*d12)) > ANGLE_TOL)
				continue;
			end
			angle_is_good = true;
			grasp_points_face_j = grasp_points_face_j_samples(:, s);
			break;
		end

		if ~angle_is_good
			continue;
		end

		% [grasp_points_face_j, in] = projectOntoTri(p2(:,1),p2(:,2),p2(:,3), grasp_points_face_i);
		% if ~in
		% 	continue;
		% end

		% projection is good, check distance to COM
		a        = norm(mesh.COM - grasp_points_face_i);
		b        = norm(mesh.COM - grasp_points_face_j);
		c        = norm(grasp_points_face_i - grasp_points_face_j);
		p        = (a+b+c)/2;
		area     = sqrt(p*(p-a)*(p-b)*(p-c));
		dist2COM = area*2/c;
		if dist2COM > para.COM_DIST_LIMIT
			continue;
		end

		% distance is good, do collision detection
		ccstart                 = tic;
		[grasp_feasible_range_pp, grasp_frame_pp] = gripperCollisionCheck(mesh, mesh_s, gripper, [grasp_points_face_i grasp_points_face_j], para);
		cctime                  = cctime + toc(ccstart);
		if isempty(grasp_feasible_range_pp)
			continue;
		end

		grasp_points(:, grasps_count, 1)      = grasp_points_face_i;
		grasp_points(:, grasps_count, 2)      = grasp_points_face_j;
		grasp_feasible_range(:, grasps_count) = grasp_feasible_range_pp;
		grasp_frame(:, grasps_count)          = grasp_frame_pp;
		fprintf(' - 	Grasp %-4d of %-4d\r', grasps_count, NGS);
		grasps_count = grasps_count + 1;
	end

	if grasps_count > NGS 
		break;
	end
end
sgtime = toc;
disp(['[CalGrasp] Grasp Sampling time:' num2str(sgtime)]);
disp(['[CalGrasp] Collision Checking time:' num2str(cctime)]);

disp(['[CalGrasp] Number of candidate grasps: ' num2str(grasps_count-1)]);

% calculate quaternions for each grasp
grasp_quats = zeros(4, NGS);
for i = 1:NGS
	v                = grasp_points(:,i,1) - grasp_points(:,i,2);
	grasp_quats(:,i) = quatBTVec(v,[1 0 0]');
end



grasps.count     = NGS;
grasps.points    = grasp_points;
grasps.quats     = grasp_quats;
grasps.range     = grasp_feasible_range;
grasps.ref_frame = grasp_frame; % describes where is the 0 in grasps.range

% --------------------------------------------
% 		Calculate feasible grasps 
% 		for each stable placement
% --------------------------------------------
disp('[CalGrasp] Computing feasible grasps for each stable placement:');
m_grasps = ones(fgraph.NM, NGS);
for i = 1:fgraph.NM
	if para.showStablePoses
		plotObject(mesh, para.showStablePoses_id, fgraph.quat(:,i));
    end

    m_grasps(i,:) = checkGraspPoints(grasps, mesh, pgraph, fgraph.quat(:,i), para.showGraspChecking_id, para);
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
