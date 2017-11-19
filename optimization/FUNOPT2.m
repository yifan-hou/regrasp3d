function FUN = FUNOPT2(x, para)

nq                 = x;
para_delta_theta   = para.delta_theta;
para_q0            = para.q0;
para_qf            = para.qf;
para_Gp1o          = para.Gp1o;
para_Gp2o          = para.Gp2o;
% para_GpZ_limit     = para.GpZ_limit;
para_Gp_tilt_limit = para.Gp_tilt_limit;
para_cost_goal_k   = para.cost_goal_k;
para_cost_tilt_k   = para.cost_tilt_k;

N = length(x)/3;

% Numerical
nq_ = zeros(3, N);
for i = 1:N
	nq_(1, i) = nq((i-1)*3+1);
	nq_(2, i) = nq((i-1)*3+2);
	nq_(3, i) = nq((i-1)*3+3);
end
theta                  = normByCol(nq_);
temp                   = ones(3,1)*(sin(theta)./theta);
q                      = [cos(theta); temp.*nq_];
q_plus                 = q(:, 2:end);
q_minus                = q(:, 1:end-1);
qq                     = diag(q_plus'*q_minus);
CON_L_obj_ang_bt_frame = acos(qq) - para_delta_q; % < 0

% Workspace
Gp1 = zeros(3, N);
Gp2 = zeros(3, N);
for i = 1:N
	Gp1(:, i) = quatOnVec(para_Gp1o, q(:, i));
	Gp2(:, i) = quatOnVec(para_Gp2o, q(:, i));
end
Gax          = (Gp1 - Gp2);
Gax_tilt_ang = zeros(N, 1);
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
dist_2_goal = zeros(1, N);
for i = 1:N
	dist_2_goal(i) = acos(para_qf'*q(:,i));
end
COST_GOAL = sum(dist_2_goal);


% summary
COST  = para_cost_goal_k*COST_GOAL + para_cost_tilt_k*COST_tilt;
CON_L = [CON_L_obj_ang_bt_frame; CON_L_tilt];
CON_E = [CON_E_q_init; CON_E_q_final];

FUN = [COST; CON_L; CON_E];