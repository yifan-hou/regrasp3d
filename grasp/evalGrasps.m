% evaluate the grasps for current mode trasition
% input:
% 	midin: scalar, id of input mode.
% 	midout: scalar, id of output mode.
% 	m_type: M x 1 vector, mode type of each mode.
%	m_is_stable: M x 1 boolean vector. stability of each mode.
% 	grasp_points: 3 x N x 2. list of grasp points.
% 	midpre (optional): scalar, id of previous mode. 
% 			is required when midin is a vertex.
% output:
%	grasp_eval:	N x 1 boolean vector.  
function [grasp_eval] = evalGrasps(midin, midout, mode_graph, mesh, grasp_points, midpre)
global M_TYPE_VERTEX M_TYPE_EDGE M_TYPE_FACET
global LIFT_TOLERANCE GRIPPER_TILT_LIMIT GRIPPER_Z_LIMIT

grasp_eval = ones(N, 1);
N          = size(grasp_points, 2);

% ------------------------------------------
% 		Pulling up
% ------------------------------------------
% just check 
if mode_graph.node.type(midin) == M_TYPE_FACET
	% get facet normal n and the transformation that rotates n to [0 0 1]
	n = mode_graph.node.faceNormal{midin};
	qn = quatBTVec(n, [0 0 1]');

	% rotates all the grasp points and check
	for i = 1:N
		grasp_points_rot(:,1) = quatOnVec(grasp_points(:,i,1), qn);
		grasp_points_rot(:,2) = quatOnVec(grasp_points(:,i,2), qn);
		grasp_axis_i = grasp_points_rot(:,1) - grasp_points_rot(:,2);
		if abs(grasp_axis_i(3))/norm(grasp_axis_i) > sin(GRIPPER_TILT_LIMIT)
			grasp_eval(i) = 0;
		elseif min(grasp_points_rot(3,:)) < GRIPPER_Z_LIMIT
			grasp_eval(i) = 0;
		else
			grasp_eval(i) = 1;
		end
	end
	return;
end % midin = M_TYPE_FACET


if (mode_graph.node.type(midin) == M_TYPE_EDGE)&&(mode_graph.node.type(midout) = M_TYPE_VERTEX)
	grasp_eval = ones(N,1); % if already grasped by edge, then the lifting is always possible
end

% ------------------------------------------
% 		Putting down
% ------------------------------------------
if (mode_graph.node.type(midin) == M_TYPE_VERTEX)&&(mode_graph.node.type(midout) == M_TYPE_EDGE)
	pvertex = mode_graph.node.contact_area{midin}; % should be 3x1
	pedge   = mode_graph.node.contact_area{midout}; % should be 3x2
	if norm(pedge(:,1) - pvertex) > 1e-8 % make sure pedge(:,1) = pvertex
		temp = pedge(:,1);
		pedge(:,1) = pedge(:,2);
		pedge(:,2) = temp;
	end

	fcenter = [mode_graph.node.faceCenter{mid_facet(1)} mode_graph.node.faceCenter{mid_facet(2)}];
	grasp_direction = reshape(grasp_points(:,:,1) - grasp_points(:,:,2), [3,N]);
	grasp_direction = bsxfun(@rdivide, grasp_direction, normByCol(grasp_direction));

	if mode_graph.node.type(midpre) == M_TYPE_EDGE
		% todo: handle this situation
		grasp_eval = zeros(N,1);
	elseif mode_graph.node.type(midpre) = M_TYPE_FACET
		% get facet normal n and the transformation that rotates n to [0 0 1]
		n  = mode_graph.node.faceNormal{midpre};
		qn = quatBTVec(n, [0 0 1]');

		% find the two facets next to the edge midout
		id_list = find(mode_graph.connectmatrix(midout,:));
		id_facet = find(mode_graph.node.type(id_list) == M_TYPE_FACET);
		mid_facet = id_list(id_facet);
		if length(mid_facet)~=2
			error('Something is wrong here');
		end
		% find their intersection with z = 0 plane
		n1  = mode_graph.node.faceNormal{mid_facet(1)};
		n2  = mode_graph.node.faceNormal{mid_facet(2)};
		n1p = quatOnVec(n1, qb);
		n2p = quatOnVec(n2, qb);
		itsc1p      = cross([0 0 1]', n1p); % intersection
		itsc2p      = cross([0 0 1]', n2p);
		pvertexp      = quatOnVec(pvertex, qb);
		pedgep(:,1)   = quatOnVec(pedge(:,1), qb); % should = pvertexp
		pedgep(:,2)   = quatOnVec(pedge(:,2), qb);
		fcenterp(:,1) = quatOnVec(fcenter(:,1), qb);
		fcenterp(:,2) = quatOnVec(fcenter(:,2), qb);
		if cross(pedgep(:,1) - pedgep(:,2), itsc1p)'*cross(pedgep(:,1) - pedgep(:,2), fcenterp(:,1) - pvertexp) < 0
			itsc1p = - itsc1p;
		end
		if cross(pedgep(:,1) - pedgep(:,2), itsc2p)'*cross(pedgep(:,1) - pedgep(:,2), fcenterp(:,2) - pvertexp) < 0
			itsc2p = - itsc2p;
		end

		% rotates edge to plane, check feasibility
		for i = 1:N
			ax = grasp_direction(:,i);
			edge = pedgep(:,2) - pedgep(:,1);
			
			a = edge(3)/2*ax(1)^2 -edge(1)*ax(1)*ax(3) + edge(3)/2*ax(2)^2 - edge(2)*ax(2)*ax(3) - edge(3)/2*ax(3)^2 + edge(3)/2;
			b = edge(2)*ax(1) - edge(1)*ax(2);
			c = -edge(3)/2*ax(1)^2 + edge(1)*ax(1)*ax(3) - edge(3)/2*ax(2)^2 + edge(2)*ax(2)*ax(3) + edge(3)/2*ax(3)^2 + edge(3)/2;

			% acos + bsin = -c
			phi = atan(a,b);
			k   = sqrt(a^2 + b^2);
			theta = asin(-c/k) - phi;
			qrot = [cos(theta/2); ax*sin(theta/2)];
			testedge = quatOnVec(edge, qrot);
			if abs(testedge(3)) > 1e-7
				qrot = [cos((theta+pi)/2); ax*sin((theta+pi)/2)];
			end

			% rotate itsc1p and itsc2p, check
			itsc1pp = quatOnVec(itsc1p, qrot);
			itsc2pp = quatOnVec(itsc2p, qrot);
			if (itsc1pp(3) < 0) || (itsc2pp(3) < 0)
				grasp_eval(i) = 0;
			else
				grasp_eval(i) = 1;
			end
		end
	else
		error('midpre can not be vertex');
	end
	return;
end

if (mode_graph.node.type(midin) == M_TYPE_VERTEX)&&(mode_graph.node.type(midout) == M_TYPE_FACET)
	% todo: think about this situation
	% 		it is not robust, so we disable it for now
	grasp_eval = zeros(N,1);
end

if (mode_graph.node.type(midin) == M_TYPE_EDGE)&&(mode_graph.node.type(midout) == M_TYPE_FACET)
	pedge = mode_graph.node.contact_area{midin};
	edge = pedge(:,1) - pedge(:,2);

	% find the two facets next to the edge midin
	id_list   = find(mode_graph.connectmatrix(midin,:));
	id_facet  = find(mode_graph.node.type(id_list) == M_TYPE_FACET);
	mid_facet = id_list(id_facet);
	n1        = mode_graph.node.faceNormal{mid_facet(1)};
	n2        = mode_graph.node.faceNormal{mid_facet(2)};
	q1 = quatBTVec(n1, [0 0 1]');
	q2 = quatBTVec(n2, [0 0 1]');

	for i = 1:N
		% check if grasp axis is parallel to edge
		grasp_axis_i = reshape(grasp_points(:,i,:), [3 2]);
		if norm(cross(edge, grasp_axis_i(:,1) - grasp_axis_i(:,2))) > 1e-7
			% can not do pivoting, only do rolling
			% check rolling
			% todo: handle this situation
			grasp_eval = zeros(N, 1);
		else
			% pivoting
			% change frame, n1 to [0 0 1]
			n2q = quatOnVec(n2, q1);
			COMq = quatOnVec(mesh.COM, q1);
			grasp_axis_i_q = grasp_axis_i;
			grasp_axis_i_q(:,1) = quatOnVec(grasp_axis_i(;,1), q1);
			grasp_axis_i_q(:,2) = quatOnVec(grasp_axis_i(;,2), q1);
			edgeq = edge;
			edgeq(:,1) = quatOnVec(edge(:,1), q1);
			edgeq(:,2) = quatOnVec(edge(:,2), q1);

			% change frame to 2D problem, looking from grasp axis direction
			qg                  = quatBTVec(grasp_axis_i_q(:,1) - grasp_axis_i_q(:,2), [0 1 0]); % this should be a rotation about z axis only
			n1q                 = quatOnVec([0 0 1]', qg);
			n2q                 = quatOnVec(n2q, qg);
			COMq                = quatOnVec(COMq, qg);
			grasp_axis_i_q(:,1) = quatOnVec(grasp_axis_i_q(;,1), qg);
			grasp_axis_i_q(:,2) = quatOnVec(grasp_axis_i_q(;,2), qg);
			edgeq(:,1)          = quatOnVec(edgeq(:,1), qg);
			edgeq(:,2)          = quatOnVec(edgeq(:,2), qg);

			% calculate the rotation between starting and ending pose

			
			% calculate the dangerous zone angle
		end


end


% Check rotation direction under gravity.
% Gravity direction is assumed to be [0 0 -1]
% input:
% 	COM: 3 x 1, position of COM
% 	ax: 3 x 1 or 3 x 2, rotation vertex or rotation axis
% 	gp: 3 x 2, grasp position
% 	dir: boolean, desired object rotation direction w.r.t. gp 
% output:
% 	isok: scalar, 
% 		+1: good
% 		0: 
function isok = checkGravity(COM, ax, gp, dir)
vgp = gp(:,1) - gp(:,2);

pCOM = prjP2Line(gp(:,1), gp(:,2), COM);
pax1 = prjP2Line(gp(:,1), gp(:,2), ax(:,1));

vCOM = COM - pCOM;
vax1 = ax(:,1) - pax1;
if size(ax,2) > 1
	pax2 = prjP2Line(gp(:,1), gp(:,2), ax(:,2));
	vax2 = ax(:,2) - pax2;
end

isok = true;
g = [0 0 -1]';
% check gravity
Gf = cross(vCOM, g);
if Gf'*vgp*dir < 1e-7
	isok = false;
end
% check contact force
Rf1 = cross(vax1, g);
if Rf1'*vgp*dir < 1e-7
	isok = false;
end
if size(ax,2) > 1
	Rf2 = cross(vax2, g);
	if Rf2'*vgp*dir < 1e-7
		isok = false;
	end
end


end





function isok = checkFingerPoseOnFacet(gp, mid, m_type)
