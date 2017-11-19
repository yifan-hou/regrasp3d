clear;clc;

% hyper parameters
N = 50; % # of frames


% variables
nq = sym('nq', [3*N, 1], 'real'); % object orientation

% parameters
para_delta_q       = sym('para_delta_q', 'real');
para_q0            = sym('para_q0', [4, 1], 'real');
para_qf            = sym('para_qf', [4, 1], 'real');
para_Gp1o          = sym('para_Gp1o', [3, 1], 'real');
para_Gp2o          = sym('para_Gp2o', [3, 1], 'real');
para_GpZ_limit     = sym('para_GpZ_limit', 'real');
para_Gp_tilt_limit = sym('para_Gp_tilt_limit', 'real');

para_cost_goal_k = sym('para_cost_goal_k', 'real');
para_cost_tilt_k = sym('para_cost_tilt_k', 'real');



% --------------------------------
% 	constraints
% 		CON_L_ : < 0
% 		CON_E_ : = 0
% 	costs
% 		COST_
% --------------------------------
% bound
xlow = -2*pi*ones(3*N, 1);
xupp = 2*pi*ones(3*N, 1);

% Numerical
nq_ = sym(zeros(3, N));
for i = 1:N
	nq_(1, i) = nq((i-1)*3+1);
	nq_(2, i) = nq((i-1)*3+2);
	nq_(3, i) = nq((i-1)*3+3);
end
theta                  = normByCol(nq_);
temp                   = ones(3,1)*(sin(theta)./theta);
q                      = [cos(theta); temp.*nq_];
q                      = simplify(q);
q_plus                 = q(:, 2:end);
q_minus                = q(:, 1:end-1);
qq                     = diag(q_plus'*q_minus);
qq                     = simplify(qq);
CON_L_obj_ang_bt_frame = acos(qq) - para_delta_q; % < 0

% Workspace
Gp1 = sym(zeros(3, N));
Gp2 = sym(zeros(3, N));
for i = 1:N
	Gp1(:, i) = quatOnVec(para_Gp1o, q(:, i));
	Gp2(:, i) = quatOnVec(para_Gp2o, q(:, i));
end
CON_L_GpZ = [para_GpZ_limit - Gp1(3, :)'; ...
			 para_GpZ_limit - Gp2(3, :)'];
% Gaxnorm      = ones(3,1)*normByCol(Gp1 - Gp2);
Gax          = (Gp1 - Gp2);
Gax_tilt_ang = sym(zeros(N, 1));
for i = 1:N
	Gax_tilt_ang(i) = angBTVec([0; 0; 1], Gax(:, i));
end
Gax_tilt_ang_horizontal = (Gax_tilt_ang - pi/2).^2;
CON_L_tilt              = Gax_tilt_ang_horizontal - para_Gp_tilt_limit^2;
COST_tilt               = sum(Gax_tilt_ang_horizontal);


% Boundary
q_init          = q(:, 1);
q_final         = q(:, end);
CON_E_q_init    = acos(para_q0'*q_init); % = 0
% CON_E_q_final   = acos(para_qf);


[~, q_final_ax] = quat2aa(quatMTimes(quatInv(para_qf), q_final));
CON_E_q_final   = norm([0 0 1]' - q_final_ax);


% goal
dist_2_goal = sym(zeros(1, N));
for i = 1:N
	dist_2_goal(i) = acos(para_qf'*q(:,i));
end
COST_GOAL = sum(dist_2_goal);


% summary
COST  = para_cost_goal_k*COST_GOAL + para_cost_tilt_k*COST_tilt;
CON_L = [CON_L_obj_ang_bt_frame; CON_L_GpZ; CON_L_tilt];
CON_E = [CON_E_q_init; CON_E_q_final];

FUN = [COST; CON_L; CON_E];

Flow = [-inf; -inf(size(CON_L)); zeros(size(CON_E))];
Fupp = [inf; zeros(size(CON_L)); zeros(size(CON_E))];


% --------------------------------
% 	Derivatives
% --------------------------------
G = sym(zeros(size(FUN, 1), 3*N));

for i = 1:3*N
	name    = ['np' num2str(i)];
	G(:, i) = diff(FUN, name);
end


% --------------------------------
% 	Construct the function
% --------------------------------
matlabFunction(Flow, 'File', 'generated/Flow1');
matlabFunction(Fupp, 'File', 'generated/Fupp1');
matlabFunction(xlow, 'File', 'generated/xlow1');
matlabFunction(xupp, 'File', 'generated/xupp1');

matlabFunction(FUN, 'File', 'generated/FUN1', 'vars',{nq, para_delta_q, para_q0, para_qf, para_Gp1o, para_Gp2o, para_GpZ_limit, para_Gp_tilt_limit, para_cost_goal_k, para_cost_tilt_k});
matlabFunction(G, 'File', 'generated/GRAD1', 'vars',{nq, para_delta_q, para_q0, para_qf, para_Gp1o, para_Gp2o, para_GpZ_limit, para_Gp_tilt_limit, para_cost_goal_k, para_cost_tilt_k});




