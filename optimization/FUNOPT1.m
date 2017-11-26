% pose to pose
function FUN = FUNOPT1(x, para)

nq                 = x;
para_delta_theta   = para.delta_theta;
para_q0            = para.q0;
para_qf            = para.qf;
para_qf_inv		   = para.qf_inv;
para_Gp1o          = para.Gp1o;
para_Gp2o          = para.Gp2o;
% para_GpZ_limit     = para.GpZ_limit;
para_Gp_tilt_limit = para.Gp_tilt_limit;
para_cost_goal_k   = para.cost_goal_k;
para_cost_tilt_k   = para.cost_tilt_k;

N = length(x)/3;

% Numerical
% nq_ = zeros(3, N);
% for i = 1:N
% 	nq_(1, i) = nq((i-1)*3+1);
% 	nq_(2, i) = nq((i-1)*3+2);
% 	nq_(3, i) = nq((i-1)*3+3);
% end
nq_ = reshape(nq, [3,N]);
theta                  = normByCol(nq_);
temp                   = ones(3,1)*(sin(theta)./theta);
q                      = [cos(theta); temp.*nq_];
for i = 1:N
	if theta(i) < 1e-8
		q(:, i) = [1 0 0 0]';
	end
end		
q_plus                 = q(:, 2:end);
q_minus                = q(:, 1:end-1);
qq                     = sum(q_plus.*q_minus)';
CON_L_obj_ang_bt_frame = acos(qq) - para_delta_theta; % < 0

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
para_Gp1o_ = A*para_Gp1o;
tempA1     = quatMTimes_N1(q, para_Gp1o_);
tempB1     = quatInv(q);
Gp1        = quatMTimes_NN(tempA1, tempB1);

para_Gp2o_ = A*para_Gp2o;
tempA2     = quatMTimes_N1(q, para_Gp2o_);
tempB2     = quatInv(q);
Gp2        = quatMTimes_NN(tempA2, tempB2);

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
COST_GOAL = sum(dist_2_goal);


% summary
COST  = para_cost_goal_k*COST_GOAL + para_cost_tilt_k*COST_tilt;
CON_L = [CON_L_obj_ang_bt_frame; CON_L_tilt];
CON_E = [CON_E_q_init; CON_E_q_final];

FUN = [COST; CON_L; CON_E];


% return;

Flow = [-inf; -inf(size(CON_L)); zeros(size(CON_E))];
Fupp = [inf; zeros(size(CON_L)); zeros(size(CON_E))];
xlow = -2*pi*ones(3*N, 1);
xupp = 2*pi*ones(3*N, 1);

save snoptfiles/sizeInfo1.mat Flow Fupp xlow xupp

end