function [grasps, fgraph] = checkGrasp4StableMode(fgraph, pgraph, mesh, para)
load clickData.mat
% --------------------------------------------
% 		Calculate feasible grasps 
% 		for each stable placement
% --------------------------------------------
disp('[CalGrasp] Computing feasible grasps for each stable placement:');
m_grasps = ones(fgraph.NM, grasps.count);
for i = 1:fgraph.NM
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
