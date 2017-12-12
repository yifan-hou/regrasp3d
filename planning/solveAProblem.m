function [path_found, plan] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f, method)
global para fgraph pgraph mesh gripper grasps % inputs

addpath ../optimization/autodiff_generated
addpath ../optimization/snoptfiles
addpath ../optimization/

plan = [];
% treat initial/final pose as additional mode
% build the full graph
dispC('[Planning] Building Full Graph:');
adj_matrix = zeros(fgraph.NM+2);
adj_matrix(1:fgraph.NM, 1:fgraph.NM) = fgraph.adj_matrix;

% get grasp points/grasp angles for initial and final pose
if isempty(grasp_id_0)
	grasp_id_0 = checkGraspPoints(grasps, mesh, pgraph, q0, para.showGraspChecking_id(1), para);
end
if isempty(grasp_id_f)
	grasp_id_f = checkGraspPoints(grasps, mesh, pgraph, qf, para.showGraspChecking_id(2), para);
end

for m = 1:fgraph.NM
	if any(grasp_id_0&fgraph.grasps(m,:))
		adj_matrix(m, fgraph.NM+1) = 1;
		adj_matrix(fgraph.NM+1, m) = 1;
	end
	if any(grasp_id_f&fgraph.grasps(m,:))
		adj_matrix(m, fgraph.NM+2) = 1;
		adj_matrix(fgraph.NM+2, m) = 1;
	end
end
adj_matrix(fgraph.NM+1, fgraph.NM+1) = 1;
adj_matrix(fgraph.NM+2, fgraph.NM+2) = 1;
dispC('Adjacent Matrix:');
dispC(adj_matrix);
if any(grasp_id_f&grasp_id_0)
	adj_matrix(fgraph.NM+1, fgraph.NM+2) = 1;
	adj_matrix(fgraph.NM+2, fgraph.NM+1) = 1;
end
if ~any(grasp_id_0)
	path_found = false;
	dispC('[Planning] No Solution. No feasible initial grasps.');
	return;
end
if ~any(grasp_id_f)
	path_found = false;
	dispC('[Planning] No Solution. No feasible final grasps.');
	return;
end

mode_grasps  = [fgraph.grasps; grasp_id_0; grasp_id_f];
path_counter = 1;
while true
	[~, mode_id_path] = dijkstra(adj_matrix, ones(fgraph.NM+2), fgraph.NM+1, fgraph.NM+2);
	NP                = length(mode_id_path);
	path_found        = false;
	if isnan(mode_id_path)
		dispC('[Planning] No solution found given the available grasps.');
		break;
	end
	dispC(['[Planning] Trying Path #' num2str(path_counter) ', Path length = ' num2str(NP) ]);
	dispC('[Planning] Planning for each edge on the path: ');

	path_q       = zeros(4, NP);
	path_graspid = zeros(1, NP-1);
	plan         = cell(1,  NP-1);

	% motion planning for each edge on the path
	path_q(:,1) = q0;
	for p = 1:NP-1
		if p == 1
			qg0_p = qg0;
		else
			qg0_p = [];
		end

		exactq = false;
		if p == NP-1
			qgf_p         = qgf;
			path_q(:,p+1) = qf;
			exactq        = true;
		else
			qgf_p         = [];
			path_q(:,p+1) = getClosestQuat(path_q(:, p), fgraph.quat(:, mode_id_path(p+1)), [0 0 1]');
		end

		id_common = find(mode_grasps(mode_id_path(p),:) & mode_grasps(mode_id_path(p+1),:)); 
		assert(~isempty(id_common)); % if failed, the graph search has problem
		dispC(['  Edge #' num2str(p) ', common grasps: ' num2str(length(id_common))]);

% 		% ploting
% 		gp1o_w = grasps.points(:, id_common, 1);
% 		gp2o_w = grasps.points(:, id_common, 2);
% 		gp10 = quatOnVec(gp1o_w, path_q(:,p));
% 		plotObject(mesh,2,path_q(:,p));
% 		gp20 = quatOnVec(gp2o_w, path_q(:,p));
% 		hold on; plot3(gp10(1,:), gp10(2,:), gp10(3,:), '.', 'markersize',35);
% 		hold on; plot3(gp20(1,:), gp20(2,:), gp20(3,:), '.', 'markersize',35);
% 		plotObject(mesh,3,path_q(:,p+1));
% 		gp1f = quatOnVec(gp1o_w, path_q(:,p+1));
% 		gp2f = quatOnVec(gp2o_w, path_q(:,p+1));
% 		hold on; plot3(gp1f(1,:), gp1f(2,:), gp1f(3,:), '.', 'markersize',35);
% 		hold on; plot3(gp2f(1,:), gp2f(2,:), gp2f(3,:), '.', 'markersize',35);
% 		input('Press ENTER to continue...');


		for i = 1:length(id_common)
			path_graspid(p) = id_common(i);

			if strcmp(method, 'pickplace')
				[plan_one_grasp, flag] = planOneGraspPickPlace(id_common(i), path_q(:,p), path_q(:,p+1), qg0_p, qgf_p);
			else
				[plan_one_grasp, flag] = planOneGrasp(id_common(i), path_q(:,p), path_q(:,p+1), qg0_p, qgf_p, exactq);
			end

		    if flag <= 0
                switch flag
                    case -1
                        dispC(['  --- Grasp ' num2str(i) ' [Pre-checking] initial/final gripper collides with table']);
                    case -2
                        dispC(['  --- Grasp ' num2str(i) ' [Pre-checking] initial/final grasp infeasible in cf_range']);
                    case -3
                        dispC(['  --- Grasp ' num2str(i) ' [q obj optimization] violates gripper tilt angle limit ']);
                    case -4
                        dispC(['  --- Grasp ' num2str(i) ' [q obj optimization] violates gripper Z limit ']);
                    case -5
                        dispC(['  --- Grasp ' num2str(i) ' [q obj optimization] infeasible ']);
                    case -6
                        dispC(['  --- Grasp ' num2str(i) ' [q obj checking] no collision free path ']);
                    case -7
                        dispC(['  --- Grasp ' num2str(i) ' [QP for GRP] infeasible ']);
                    otherwise
                        error('Wrong flag');
                end
		        continue;
		    else
				plan{p}        = plan_one_grasp;
				path_q(:, p+1) = plan_one_grasp.qobj(:, end);
		        dispC(['  --- Grasp ' num2str(i) ' works.']);
		        break;
		    end
			% gripper motion closed loop control
		end

		if isempty(plan{p})
			dispC(['  Edge #' num2str(p) ' No solution.']);
			adj_matrix(mode_id_path(p), mode_id_path(p+1)) = 0;
			adj_matrix(mode_id_path(p+1), mode_id_path(p)) = 0;
			break;
		elseif p == NP-1
			path_found = true;
		end

	end % end a path

	if path_found
		dispC(['[Planning] Solution found. Length = ' num2str(NP) ]);
		% print to file
		
		break;
	end

end % end graph search
