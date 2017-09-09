function [path_found, path_q, path_graspid, path_qp, plan_2d] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f)
global para fgraph pgraph mesh gripper grasps % inputs

% treat initial/final pose as additional mode
% build the full graph
disp('[Planning] Building Full Graph:');
adj_matrix = zeros(fgraph.NM+2);
adj_matrix(1:fgraph.NM, 1:fgraph.NM) = fgraph.adj_matrix;
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
disp('Adjacent Matrix:');
disp(adj_matrix);
if any(grasp_id_f&grasp_id_0)
	adj_matrix(fgraph.NM+1, fgraph.NM+2) = 1;
	adj_matrix(fgraph.NM+2, fgraph.NM+1) = 1;
end
if ~any(grasp_id_0)
	set(handles.BTN_animate, 'Enable', 'off');
	disp('[Planning] No Solution. No feasible initial grasps.');
	return;
end
if ~any(grasp_id_f)
	set(handles.BTN_animate, 'Enable', 'off');
	disp('[Planning] No Solution. No feasible final grasps.');
	return;
end

mode_grasps  = [fgraph.grasps; grasp_id_0; grasp_id_f];
path_counter = 1;
while true
	[~, mode_id_path] = dijkstra(adj_matrix, ones(fgraph.NM+2), fgraph.NM+1, fgraph.NM+2);
	NP                = length(mode_id_path);
	if isnan(mode_id_path)
		disp('[Planning] No solution found given the available grasps.');
		break;
	end
	disp(['[Planning] Trying Path #' num2str(path_counter) ', Path length = ' num2str(NP) ]);
	disp('[Planning] Planning for each edge on the path: ');

	path_q       = zeros(4, NP);
	path_graspid = zeros(1, NP-1);
	path_qp      = zeros(4, NP-1);
	plan_2d      = cell(1,  NP-1);
	path_found   = false;

	% motion planning for each edge on the path
	path_q(:,1) = q0;
	for p = 1:NP-1
		if p == 1
			qg0_p = qg0;
		else
			qg0_p = [];
		end

		if p == NP-1
			qgf_p         = qgf;
			path_q(:,p+1) = qf;
		else
			qgf_p         = [];
			path_q(:,p+1) = fgraph.quat(:, mode_id_path(p+1));
		end
		id_common = find(mode_grasps(mode_id_path(p),:) & mode_grasps(mode_id_path(p+1),:)); 
		assert(~isempty(id_common)); % if failed, the graph search has problem
		disp(['  Edge #' num2str(p) ', common grasps: ' num2str(length(id_common))]);
		for i = 1:length(id_common)
			path_graspid(p) = id_common(i);

			[plan_2d_temp, qp_temp, flag] = planOneGrasp(mesh, grasps, id_common(i), path_q(:,p), path_q(:,p+1), qg0_p, qgf_p, pgraph, para);

		    if flag <= 0
                switch flag
                    case -1
                        disp(['  --- Grasp ' num2str(i) ' Required rolling exceeds gripper tilt limit']);
                    case -3
                        disp(['  --- Grasp ' num2str(i) ' Initial/final grasp pos violates gripper tilt limit ']);
                    case -4
                        disp(['  --- Grasp ' num2str(i) ' Initial/final grasp pos violates gripper Z limit ']);
                    case -5
                        disp(['  --- Grasp ' num2str(i) ' Gripper motion infeasible ']);
                    otherwise
                        error('Wrong flag');
                end
		        continue;
		    else
				plan_2d{p} = plan_2d_temp;
				path_qp(:,p)            = qp_temp;
		        disp(['  --- Grasp ' num2str(i) ' works.']);
		        break;
		    end
			% gripper motion closed loop control
		end

		if isempty(plan_2d{p})
			disp(['  Edge #' num2str(p) ' No solution.']);
			adj_matrix(mode_id_path(p), mode_id_path(p+1)) = 0;
			adj_matrix(mode_id_path(p+1), mode_id_path(p)) = 0;
			break;
		elseif p == NP-1
			path_found = true;
		end

	end % end a path

	if path_found
		disp(['[Planning] Solution found. Length = ' num2str(NP) ]);
		break;
	end

end % end graph search