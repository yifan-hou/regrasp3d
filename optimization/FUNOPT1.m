function [f, g] = FUNOPT1(x, para)

n                  = x;
para_delta_theta       = para.delta_theta;
para_q0            = para.q0;
para_qf            = para.qf;
para_Gp1o          = para.Gp1o;
para_Gp2o          = para.Gp2o;
para_GpZ_limit     = para.GpZ_limit;
para_Gp_tilt_limit = para.Gp_tilt_limit;
para_cost_goal_k   = para.cost_goal_k;
para_cost_tilt_k   = para.cost_tilt_k;

N = length(x)/3;
% --------------------------------
% 	constraints
% 		CON_L_ : < 0
% 		CON_E_ : = 0
% 	costs
% 		COST_
% --------------------------------
n_ = zeros(3, N);
for i = 1:N
	n_(1, i) = n((i-1)*3+1);
	n_(2, i) = n((i-1)*3+2);
	n_(3, i) = n((i-1)*3+3);
end

M0   = quat2m(para_q0);
M    = zeros(3,3,N);
Macc = zeros(3,3,N);
for i = 1:N
	M(:,:,i) = exp2m(n_(:, i));
	if i == 1
		Macc(:,:,i) = M0;
	else
		Macc(:,:,i) = M(:,:,i)*Macc(:,:,i-1);
	end
end

% Numerical
CON_L_obj_ang_bt_frame     = normByCol(n_)'.^2 - para_delta_theta^2; % < 0
CON_gradq_obj_ang_bt_frame = sparse(zeros(N, 3*N));
for i = 1:N
	CON_gradq_obj_ang_bt_frame(i, (i-1)*3+1:i*3) = n_(:, i)';
end

% Workspace
Gp1 = reshape(ten_mat(Macc, para_Gp1o), [3, N]);
Gp2 = reshape(ten_mat(Macc, para_Gp2o), [3, N]);

Gax          = (Gp1 - Gp2);
Gax_tilt_ang = zeros(N, 1);
for i = 1:N
	Gax_tilt_ang(i) = angBTVec([0; 0; 1], Gax(:, i));
end
tilt2 = (Gax_tilt_ang - pi/2).^2;
CON_L_tilt              = tilt2 - para_Gp_tilt_limit^2;
CON_gradq_tilt = sparse(zeros(N, 4*N));





COST_tilt               = sum(tilt2);


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






end