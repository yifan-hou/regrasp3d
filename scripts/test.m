clfAll;clear;clc;


addpath ../grasp
addpath ../model

% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit
% friction between object and  ground
para.MU = 0.5;
% grasp pos sample density
para.GS_DENSITY = 0.1; % 1 point every 0.1 m^2
% grasp axis tolerance
para.ANGLE_TOL = 0.1; % rad
para.COM_DIST_LIMIT = 0.8; % meter
% shape of gripper(for collision checking)
% para.gripper_shape = getGripper();
para.GOALSAMPLEDENSITY2D = 15*pi/180; % 1 sample every 5 degree
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree

para.showObject             = false;
para.showObject_id          = 1;
para.showAllGraspSamples    = false;
para.showAllGraspSamples_id = 1;
para.showCheckedGrasp       = false;
para.showCheckedGrasp_id    = 3;
para.showProblem            = true;
para.showProblem_id         = [1 2 3];
para.show2Dproblem          = false;
para.show2Dproblem_id       = 4;
para.showAnimation          = true;
para.showAnimation_id       = 5;
% get object mesh
[fgraph, pgraph, mesh] = getObject(para);

% calculate grasps, and contact mode graph
[grasps, fgraph] = calGrasp(fgraph, mesh, para);



% -----------------------------------------------
% 		Online computation
% -----------------------------------------------

% get start, goal orientation
q0 = [1 0 0 0]';
% qf = getOrientation();
% save qf.mat qf
load qf

% get grasps for initial and final pose
grasp_id_0 = checkGrasp(grasps, mesh, q0, para);
grasp_id_f = checkGrasp(grasps, mesh, qf, para);

% treat initial/final pose as additional mode
% build the full graph
connect_matrix = zeros(fgraph.NM+2);
connect_matrix(1:fgraph.NM, 1:fgraph.NM) = fgraph.connect_matrix;
for m = 1:fgraph.NM
	if any(grasp_id_0&fgraph.grasps(m,:))
		connect_matrix(m, fgraph.NM+1) = 1;
		connect_matrix(fgraph.NM+1, m) = 1;
	end
	if any(grasp_id_f&fgraph.grasps(m,:))
		connect_matrix(m, fgraph.NM+2) = 1;
		connect_matrix(fgraph.NM+2, m) = 1;
	end
end
if any(grasp_id_f&grasp_id_0)
	connect_matrix(fgraph.NM+1, fgraph.NM+2) = 1;
	connect_matrix(fgraph.NM+2, fgraph.NM+1) = 1;
end
mode_grasps = [fgraph.grasps; grasp_id_0; grasp_id_f];

path_counter = 1;
while true
	[~, mode_id_path] = dijkstra(connect_matrix, ones(fgraph.NM+2), fgraph.NM+1, fgraph.NM+2);
	% mode_id_path = graph_search(connect_matrix, fgraph.NM+1, fgraph.NM+2);
	NP           = length(mode_id_path);
	if NP == 0
		disp('[Conclusion] No solution found given the available grasps.');
		break;
	end
	disp(['[Path ' num2str(path_counter) '] length = ' num2str(NP) ]);

	path_q               = zeros(4, NP);
	path_graspid         = zeros(1, NP-1);
	path_qp              = zeros(4, NP-1);
	path_gripper_plan_2d = cell(1,  NP-1);
	path_found           = false;

	% motion planning for each edge on the path
	path_q(:,1) = q0;
	for p = 1:NP-1
		if p == NP-1
			path_q(:,p+1) = qf;
		else
			path_qf(:,p+1) = fgraph.quat(:, mode_id_path(p+1));
		end
			
		id_common = find(mode_grasps(mode_id_path(p),:) == mode_grasps(mode_id_path(p+1),:));	
		assert(~isempty(id_common));
		disp(['# Edge ' num2str(p) ', common grasps: ' num2str(length(id_common))]);

		for i = 1:length(id_common)
			path_graspid(p) = id_common(i);
			gp1o_w          = grasps.points(:,id_common(i), 1);
			gp2o_w          = grasps.points(:,id_common(i), 2);

			[griper_plan_temp, qp_temp] = planOneGrasp(mesh, gp1o_w, gp2o_w, path_q(:,p), path_q(:,p+1), pgraph, para);

		    if isempty(griper_plan_temp)
		        disp([' -- Grasp ' num2str(i) ', No solution']);
		        continue;
		    else
				path_gripper_plan_2d{p} = griper_plan_temp;
				path_qp(:,p)            = qp_temp;
		        disp([' -- Grasp ' num2str(i) ' works.']);
		        disp(path_gripper_plan_2d{p});
		        break;
		    end
			% gripper motion closed loop control
		end

		if isempty(path_gripper_plan_2d{p})
			disp(['# Edge ' num2str(p) ', No solution.']);
			connect_matrix(mode_id_path(p), mode_id_path(p+1)) = 0;
			connect_matrix(mode_id_path(p+1), mode_id_path(p)) = 0;
			break;
		elseif p == NP-1
			path_found = true;
		end

	end % end a path

	if path_found
		disp('[Conclusion] Solution found. ');
% 		disp(path_found);
		break;
	end

end % end graph search


% -----------------------------------------
% 	Animation
% -----------------------------------------
if path_found && para.showAnimation
	animatePlan(mesh, grasps, para.showAnimation_id, path_q, path_graspid, path_qp, path_gripper_plan_2d);
end