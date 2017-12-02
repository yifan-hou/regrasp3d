% pose to pose
function [FUN, Flow, Fupp, xlow, xupp] = FUNOPT_OBJ(x, para)
% function [FUN] = FUNOPT_OBJ(x, para)

nq                 = x;
para_delta_theta   = para.delta_theta;
para_q0            = para.q0;
para_qf            = para.qf;
para_qf_inv		   = para.qf_inv;
para_Gp1o          = para.Gp1o;
para_Gp2o          = para.Gp2o;
% para_GpZ_limit     = para.GpZ_limit;
para_Gp_tilt_limit = para.Gp_tilt_limit;

para_exactqf_k = para.exactqf_k;

para_cost_dq_k   = para.cost_dq_k;
para_cost_tilt_k   = para.cost_tilt_k;


N = length(x)/3;

% Numerical
nq_ = reshape(nq, [3,N]);
theta                  = normByCol(nq_);
temp                   = ones(3,1)*(sin(theta/2)./theta);
q                      = [cos(theta/2); temp.*nq_];
for i = 1:N
	if theta(i) < 1e-8
		q(:, i) = [1 0 0 0]';
	end
end		
q_plus  = q(:, 2:end);
q_minus = q(:, 1:end-1);
qq      = sum(q_plus.*q_minus)';
dq      = acos(qq);
COST_dq = sum(dq.^2);
CON_L_obj_ang_bt_frame = dq - para_delta_theta; % < 0

% Workspace 
A = [0 0 0;
	 1 0 0;
	 0 1 0;
	 0 0 1];
para_Gp1o_ = A*para_Gp1o;
tempA1     = quatMTimes_N1(q, para_Gp1o_);
tempB      = quatInv(q);
Gp1        = quatMTimes_NN(tempA1, tempB);

para_Gp2o_ = A*para_Gp2o;
tempA2     = quatMTimes_N1(q, para_Gp2o_);
Gp2        = quatMTimes_NN(tempA2, tempB);

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
CON_E_q_final = para_exactqf_k*acos(para_qf'*q_final); % = 0

qf_rot2goal       = quatMTimes_2(para_qf_inv, q_final);
[~, q_final_ax]   = quat2aa(qf_rot2goal);
CON_E_qaxis_final = (1 - para_exactqf_k)*(norm_([0 0 1]' - q_final_ax));

% % goal
% dist_2_goal = acos(para_qf'*q);
% COST_GOAL   = sum(dist_2_goal);


% summary
COST  = para_cost_dq_k*COST_dq + para_cost_tilt_k*COST_tilt;
CON_L = [CON_L_obj_ang_bt_frame; CON_L_tilt];
CON_E = [CON_E_q_init; CON_E_q_final; CON_E_qaxis_final];
FUN   = [COST; CON_L; CON_E];



Flow = [-inf; -inf(size(CON_L)); zeros(size(CON_E))];
Fupp = [inf; zeros(size(CON_L)); zeros(size(CON_E))];
xlow = -inf(3*N, 1); %2*pi*ones(3*N, 1);
xupp = inf(3*N, 1); %2*pi*ones(3*N, 1);


end