% pose to pose
function FUN = FUNOPT_GRP(x, para)
para_q             = para.q0;
para_qf            = para.qf;
% para_qf_inv		   = para.qf_inv;

N = para.N;

% Workspace
% Gp1 = zeros(3, N);
% Gp2 = zeros(3, N);
% for i = 1:N
% 	Gp1(:, i) = quatOnVec_(para_Gp1o, q(:, i));
% 	Gp2(:, i) = quatOnVec_(para_Gp2o, q(:, i));
% end
A = [0 0 0;
	 1 0 0;
	 0 1 0;
	 0 0 1];
para_Gp1o_   = A*para_Gp1o;
tempA1       = quatMTimes_N1(q, para_Gp1o_);
tempB1       = quatInv(q);
Gp1          = quatMTimes_NN(tempA1, tempB1);

para_Gp2o_   = A*para_Gp2o;
tempA2       = quatMTimes_N1(q, para_Gp2o_);
tempB2       = quatInv(q);
Gp2          = quatMTimes_NN(tempA2, tempB2);

Gax          = (Gp1 - Gp2);
tempb        = Gax./(ones(3,1)*normByCol(Gax));
Gax_tilt_ang = acos(tempb(3,:))';

Gax_tilt_ang_horizontal = (Gax_tilt_ang - pi/2).^2;
CON_L_tilt              = Gax_tilt_ang_horizontal - para_Gp_tilt_limit^2;
COST_tilt               = sum(Gax_tilt_ang_horizontal);


% Boundary
q_init        = q(:, 1);
q_final       = q(:, end);
CON_E_q_init  = acos(para_q0'*q_init); % = 0
CON_E_q_final = acos(para_qf'*q_final); % = 0

% qf_rot2goal     = quatMTimes_2(para_qf_inv, q_final);
% [~, q_final_ax] = quat2aa(qf_rot2goal);
% CON_E_q_final   = norm_([0 0 1]' - q_final_ax);

% goal
dist_2_goal = acos(para_qf'*q);
COST_GOAL   = sum(dist_2_goal);


% summary
COST  = para_cost_goal_k*COST_GOAL + para_cost_tilt_k*COST_tilt;
CON_L = [CON_L_obj_ang_bt_frame; CON_L_tilt];
CON_E = [CON_E_q_init; CON_E_q_final];

FUN = [COST; CON_L; CON_E];


% return;

Flow = [-inf; -inf(size(CON_L)); zeros(size(CON_E))];
Fupp = [inf; zeros(size(CON_L)); zeros(size(CON_E))];
xlow = -pi*ones(3*N, 1);
xupp = pi*ones(3*N, 1);

save snoptfiles/sizeInfo1.mat Flow Fupp xlow xupp

end