function [path_found, plan] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f, method)
global para fgraph pgraph mesh gripper grasps % inputs

addpath ../optimization/autodiff_generated
addpath ../optimization/snoptfiles
addpath ../optimization/

plan = [];
% treat initial/final pose as additional nodes
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

% update the connectivity matrix
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
if any(grasp_id_f&grasp_id_0)
    adj_matrix(fgraph.NM+1, fgraph.NM+2) = 1;
    adj_matrix(fgraph.NM+2, fgraph.NM+1) = 1;
end
dispC('Adjacent Matrix:');
dispC(adj_matrix);
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

% begin graph search
path_counter = 1;
while true
    [~, mode_id_path] = dijkstra(adj_matrix, ones(fgraph.NM+2), fgraph.NM+1, fgraph.NM+2);
    NP                = length(mode_id_path);
    path_found        = false;
    if isnan(mode_id_path)
        dispC('[Planning] No solution found given the available grasps.');
        break;
    end
    if NP > 4
        dispC('[Planning] No solution within 3 regrasps found given the available grasps.');
        break;
    end

    dispC(['[Planning] Trying Path #' num2str(path_counter) ', Path length = ' num2str(NP) ]);
    dispC('[Planning] Planning for each edge on the path: ');

    path_q       = zeros(4, NP);
    % path_graspid = zeros(1, NP-1);
    plan         = cell(1,  NP-1);

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
            path_q(:,p+1) = getClosestQuat(path_q(:, p), ...
                    fgraph.quat(:, mode_id_path(p+1)), [0 0 1]');
        end

        id_common = find(mode_grasps(mode_id_path(p),:) & mode_grasps(mode_id_path(p+1),:));
        assert(~isempty(id_common)); % if failed, the graph search has problem
        dispC(['  Edge #' num2str(p) ', common grasps: ' num2str(length(id_common))]);

        %
        %   Common grasps are available.
        %   Select a grasp and plan obj motion
        %
        edge_solved = true;
        [obj_plan, id_sel] = planObject(id_common, path_q(:,p), path_q(:,p+1), qg0_p, qgf_p, method);

        if isempty(id_sel)
            dispC('  --- [Object] No solution. ');
            edge_solved = false;
        else
            [plan{p}, flag] = planGripper(obj_plan, id_sel, qg0_p, qgf_p);
            if flag < 0
                edge_solved = false;
            else
                path_q(:, p+1) = plan{p}.qobj(:,end);
            end
        end

        if edge_solved == false
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
        break;
    end

end % end graph search
