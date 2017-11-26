% This code was generated using ADiGator version 1.4
% ©2010-2014 Matthew J. Weinstein and Anil V. Rao
% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ 
% Contact: mweinstein@ufl.edu
% Bugs/suggestions may be reported to the sourceforge forums
%                    DISCLAIMER
% ADiGator is a general-purpose software distributed under the GNU General
% Public License version 3.0. While the software is distributed with the
% hope that it will be useful, both the software and generated code are
% provided 'AS IS' with NO WARRANTIES OF ANY KIND and no merchantability
% or fitness for any purpose or application.

function FUN = FUNOPT2_ADiGatorJac(x,para)
global ADiGator_FUNOPT2_ADiGatorJac
if isempty(ADiGator_FUNOPT2_ADiGatorJac); ADiGator_LoadData(); end
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.FUNOPT2_ADiGatorJac.Gator1Data;
% ADiGator Start Derivative Computations
nq.dx = x.dx; nq.f = x.f;
%User Line: nq                 = x;
para_delta_theta = para.delta_theta;
%User Line: para_delta_theta   = para.delta_theta;
para_q0 = para.q0;
%User Line: para_q0            = para.q0;
para_qf = para.qf;
%User Line: para_qf            = para.qf;
para_qf_inv = para.qf_inv;
%User Line: para_qf_inv		   = para.qf_inv;
para_Gp1o = para.Gp1o;
%User Line: para_Gp1o          = para.Gp1o;
para_Gp2o = para.Gp2o;
%User Line: para_Gp2o          = para.Gp2o;
%User Line: % para_GpZ_limit     = para.GpZ_limit;
para_Gp_tilt_limit = para.Gp_tilt_limit;
%User Line: para_Gp_tilt_limit = para.Gp_tilt_limit;
para_cost_goal_k = para.cost_goal_k;
%User Line: para_cost_goal_k   = para.cost_goal_k;
para_cost_tilt_k = para.cost_tilt_k;
%User Line: para_cost_tilt_k   = para.cost_tilt_k;
cada1f1 = length(x.f);
N.f = cada1f1/3;
%User Line: N = length(x)/3;
%User Line: % Numerical
%User Line: % nq_ = zeros(3, N);
%User Line: % for i = 1:N
%User Line: % 	nq_(1, i) = nq((i-1)*3+1);
%User Line: % 	nq_(2, i) = nq((i-1)*3+2);
%User Line: % 	nq_(3, i) = nq((i-1)*3+3);
%User Line: % end
cada1f1 = [3 N.f];
nq_.dx = nq.dx;
nq_.f = reshape(nq.f,cada1f1);
%User Line: nq_ = reshape(nq, [3,N]);
cadainput2_1.dx = nq_.dx; cadainput2_1.f = nq_.f;
%User Line: cadainput2_1 = nq_;
cada1tempdx = cadainput2_1.dx;
cadainput2_1.dx = zeros(279,1);
cadainput2_1.dx(Gator1Data.Index2,1) = cada1tempdx;
cadaoutput2_1 = ADiGator_normByCol(cadainput2_1);
% Call to function: normByCol
cadaoutput2_1.dx = cadaoutput2_1.dx(Gator1Data.Index3,1);
theta.dx = cadaoutput2_1.dx; theta.f = cadaoutput2_1.f;
%User Line: theta                  = cadaoutput2_1;
cada1tf1 = theta.f(Gator1Data.Index4);
cada1f1dx = cos(cada1tf1(:)).*theta.dx;
cada1f1 = sin(theta.f);
cada1tf1 = theta.f(Gator1Data.Index5);
cada1td1 = cada1f1dx./cada1tf1(:);
cada1tf1 = cada1f1(Gator1Data.Index6);
cada1tf2 = theta.f(Gator1Data.Index7);
cada1td1 = cada1td1 + -cada1tf1(:)./cada1tf2(:).^2.*theta.dx;
cada1f2dx = cada1td1;
cada1f2 = cada1f1./theta.f;
cada1td1 = zeros(1,90);
cada1td1(Gator1Data.Index8) = cada1f2dx;
cada1td1 = Gator1Data.Data1*cada1td1;
cada1td1 = cada1td1(:);
temp.dx = cada1td1(Gator1Data.Index9);
temp.f = Gator1Data.Data1*cada1f2;
%User Line: temp                   = ones(3,1)*(sin(theta)./theta);
cada1tf1 = theta.f(Gator1Data.Index10);
cada1f1dx = -sin(cada1tf1(:)).*theta.dx;
cada1f1 = cos(theta.f);
cada1tf1 = nq_.f(Gator1Data.Index11);
cada1td1 = cada1tf1(:).*temp.dx;
cada1td1(Gator1Data.Index12) = cada1td1(Gator1Data.Index12) + temp.f(:).*nq_.dx;
cada1f2dx = cada1td1;
cada1f2 = temp.f.*nq_.f;
cada1td1 = zeros(360,1);
cada1td1(Gator1Data.Index13) = cada1f1dx;
cada1td1(Gator1Data.Index14) = cada1f2dx;
q.dx = cada1td1;
q.f = [cada1f1;cada1f2];
%User Line: q                      = [cos(theta); temp.*nq_];
cadaforvar1.f = 1:N.f;
%User Line: cadaforvar1 = 1:N;
for cadaforcount1 = 1:30
    i.f = cadaforvar1.f(:,cadaforcount1);
    %User Line: i = cadaforvar1(:,cadaforcount1);
    cada1f1 = theta.f(i.f);
    cadaconditional1 = lt(cada1f1,1e-08);
    %User Line: cadaconditional1 = theta(i) < 1e-8;
    if cadaconditional1
        q.dx(logical(Gator1Data.Index1(:,cadaforcount1))) = 0;
        q.f(:,i.f) = Gator1Data.Data2;
        %User Line: q(:, i) = [1 0 0 0]';
    end
end
cada1f1 = size(q.f,2);
cada1f2 = 2:cada1f1;
q_plus.dx = q.dx(Gator1Data.Index15);
q_plus.f = q.f(:,cada1f2);
%User Line: q_plus                 = q(:, 2:end);
cada1f1 = size(q.f,2);
cada1f2 = cada1f1 - 1;
cada1f3 = 1:cada1f2;
q_minus.dx = q.dx(Gator1Data.Index16);
q_minus.f = q.f(:,cada1f3);
%User Line: q_minus                = q(:, 1:end-1);
cada1tf1 = q_minus.f(Gator1Data.Index17);
cada1td1 = zeros(696,1);
cada1td1(Gator1Data.Index18) = cada1tf1(:).*q_plus.dx;
cada1tf1 = q_plus.f(Gator1Data.Index19);
cada1td1(Gator1Data.Index20) = cada1td1(Gator1Data.Index20) + cada1tf1(:).*q_minus.dx;
cada1f1dx = cada1td1;
cada1f1 = q_plus.f.*q_minus.f;
cada1td1 = zeros(4,174);
cada1td1(Gator1Data.Index21) = cada1f1dx;
cada1td1 = sum(cada1td1,1);
cada1f2dx = cada1td1(:);
cada1f2 = sum(cada1f1);
qq.dx = cada1f2dx;
qq.f = cada1f2.';
%User Line: qq                     = sum(q_plus.*q_minus)';
cada1tf1 = qq.f(Gator1Data.Index22);
cada1f1dx = -1./sqrt(1-cada1tf1(:).^2).*qq.dx;
cada1f1 = acos(qq.f);
CON_L_obj_ang_bt_frame.dx = cada1f1dx;
CON_L_obj_ang_bt_frame.f = cada1f1 - para_delta_theta;
%User Line: CON_L_obj_ang_bt_frame = acos(qq) - para_delta_theta;
%User Line: % Workspace
%User Line: % Gp1 = zeros(3, N);
%User Line: % Gp2 = zeros(3, N);
%User Line: % for i = 1:N
%User Line: % 	Gp1(:, i) = quatOnVec_(para_Gp1o, q(:, i));
%User Line: % 	Gp2(:, i) = quatOnVec_(para_Gp2o, q(:, i));
%User Line: % end
A.f =  [0 0 0;	 1 0 0;	 0 1 0;	 0 0 1];
%User Line: A = [0 0 0;	 1 0 0;	 0 1 0;	 0 0 1];
para_Gp1o_.f = A.f*para_Gp1o;
%User Line: para_Gp1o_ = A*para_Gp1o;
cadainput7_1.dx = q.dx; cadainput7_1.f = q.f;
%User Line: cadainput7_1 = q;
cadainput7_2.f = para_Gp1o_.f;
%User Line: cadainput7_2 = para_Gp1o_;
cadaoutput7_1 = ADiGator_quatMTimes_N1(cadainput7_1,cadainput7_2);
% Call to function: quatMTimes_N1
tempA1.dx = cadaoutput7_1.dx; tempA1.f = cadaoutput7_1.f;
%User Line: tempA1     = cadaoutput7_1;
cadainput4_1.dx = q.dx; cadainput4_1.f = q.f;
%User Line: cadainput4_1 = q;
cadaoutput4_1 = ADiGator_quatInv(cadainput4_1);
% Call to function: quatInv
tempB1.dx = cadaoutput4_1.dx; tempB1.f = cadaoutput4_1.f;
%User Line: tempB1     = cadaoutput4_1;
cadainput8_1.dx = tempA1.dx; cadainput8_1.f = tempA1.f;
%User Line: cadainput8_1 = tempA1;
cadainput8_2.dx = tempB1.dx; cadainput8_2.f = tempB1.f;
%User Line: cadainput8_2 = tempB1;
cadaoutput8_1 = ADiGator_quatMTimes_NN(cadainput8_1,cadainput8_2);
% Call to function: quatMTimes_NN
Gp1.dx = cadaoutput8_1.dx; Gp1.f = cadaoutput8_1.f;
%User Line: Gp1        = cadaoutput8_1;
para_Gp2o_.f = A.f*para_Gp2o;
%User Line: para_Gp2o_ = A*para_Gp2o;
cadainput7_1.dx = q.dx; cadainput7_1.f = q.f;
%User Line: cadainput7_1 = q;
cadainput7_2.f = para_Gp2o_.f;
%User Line: cadainput7_2 = para_Gp2o_;
cadaoutput7_1 = ADiGator_quatMTimes_N1(cadainput7_1,cadainput7_2);
% Call to function: quatMTimes_N1
tempA2.dx = cadaoutput7_1.dx; tempA2.f = cadaoutput7_1.f;
%User Line: tempA2     = cadaoutput7_1;
cadainput4_1.dx = q.dx; cadainput4_1.f = q.f;
%User Line: cadainput4_1 = q;
cadaoutput4_1 = ADiGator_quatInv(cadainput4_1);
% Call to function: quatInv
tempB2.dx = cadaoutput4_1.dx; tempB2.f = cadaoutput4_1.f;
%User Line: tempB2     = cadaoutput4_1;
cadainput8_1.dx = tempA2.dx; cadainput8_1.f = tempA2.f;
%User Line: cadainput8_1 = tempA2;
cadainput8_2.dx = tempB2.dx; cadainput8_2.f = tempB2.f;
%User Line: cadainput8_2 = tempB2;
cadaoutput8_1 = ADiGator_quatMTimes_NN(cadainput8_1,cadainput8_2);
% Call to function: quatMTimes_NN
Gp2.dx = cadaoutput8_1.dx; Gp2.f = cadaoutput8_1.f;
%User Line: Gp2        = cadaoutput8_1;
cada1td1 = Gp1.dx;
cada1td1 = cada1td1 + -Gp2.dx;
Gax.dx = cada1td1;
Gax.f = Gp1.f - Gp2.f;
%User Line: Gax          = (Gp1 - Gp2);
cadainput2_1.dx = Gax.dx; cadainput2_1.f = Gax.f;
%User Line: cadainput2_1 = Gax;
cada1tempdx = cadainput2_1.dx;
cadainput2_1.dx = zeros(279,1);
cadainput2_1.dx(Gator1Data.Index23,1) = cada1tempdx;
cadaoutput2_1 = ADiGator_normByCol(cadainput2_1);
% Call to function: normByCol
cadaoutput2_1.dx = cadaoutput2_1.dx(Gator1Data.Index24,1);
cada1td1 = zeros(1,90);
cada1td1(Gator1Data.Index25) = cadaoutput2_1.dx;
cada1td1 = Gator1Data.Data3*cada1td1;
cada1td1 = cada1td1(:);
cada1f1dx = cada1td1(Gator1Data.Index26);
cada1f1 = Gator1Data.Data3*cadaoutput2_1.f;
cada1tf1 = cada1f1(Gator1Data.Index27);
cada1td1 = Gax.dx./cada1tf1(:);
cada1tf1 = Gax.f(Gator1Data.Index28);
cada1tf2 = cada1f1(Gator1Data.Index29);
cada1td1 = cada1td1 + -cada1tf1(:)./cada1tf2(:).^2.*cada1f1dx;
tempb.dx = cada1td1;
tempb.f = Gax.f./cada1f1;
%User Line: tempb        = Gax./(ones(3,1)*cadaoutput2_1);
cada1f1dx = tempb.dx(Gator1Data.Index30);
cada1f1 = tempb.f(3,:);
cada1tf1 = cada1f1(Gator1Data.Index31);
cada1f2dx = -1./sqrt(1-cada1tf1(:).^2).*cada1f1dx;
cada1f2 = acos(cada1f1);
Gax_tilt_ang.dx = cada1f2dx;
Gax_tilt_ang.f = cada1f2.';
%User Line: Gax_tilt_ang = acos(tempb(3,:))';
cada1f1dx = Gax_tilt_ang.dx;
cada1f1 = Gax_tilt_ang.f - 1.570796326794897;
cada1tf2 = cada1f1(Gator1Data.Index32);
Gax_tilt_ang_horizontal.dx = 2.*cada1tf2(:).^(2-1).*cada1f1dx;
Gax_tilt_ang_horizontal.f = cada1f1.^2;
%User Line: Gax_tilt_ang_horizontal = (Gax_tilt_ang - pi/2).^2;
cada1f1 = para_Gp_tilt_limit^2;
CON_L_tilt.dx = Gax_tilt_ang_horizontal.dx;
CON_L_tilt.f = Gax_tilt_ang_horizontal.f - cada1f1;
%User Line: CON_L_tilt              = Gax_tilt_ang_horizontal - para_Gp_tilt_limit^2;
COST_tilt.dx = Gax_tilt_ang_horizontal.dx;
COST_tilt.f = sum(Gax_tilt_ang_horizontal.f);
%User Line: COST_tilt               = sum(Gax_tilt_ang_horizontal);
%User Line: % Boundary
q_init.dx = q.dx(Gator1Data.Index33);
q_init.f = q.f(:,1);
%User Line: q_init          = q(:, 1);
cada1f1 = size(q.f,2);
q_final.dx = q.dx(Gator1Data.Index34);
q_final.f = q.f(:,cada1f1);
%User Line: q_final         = q(:, end);
cada1f1 = para_q0.';
cada1td1 = zeros(4,3);
cada1td1(Gator1Data.Index35) = q_init.dx;
cada1td1 = cada1f1*cada1td1;
cada1td1 = cada1td1(:);
cada1f2dx = cada1td1(Gator1Data.Index36);
cada1f2 = cada1f1*q_init.f;
CON_E_q_init.dx = -1./sqrt(1-cada1f2.^2).*cada1f2dx;
CON_E_q_init.f = acos(cada1f2);
%User Line: CON_E_q_init    = acos(para_q0'*q_init);
%User Line: % CON_E_q_final   = acos(para_qf);
cadainput6_1 = para_qf_inv;
%User Line: cadainput6_1 = para_qf_inv;
cadainput6_2.dx = q_final.dx; cadainput6_2.f = q_final.f;
%User Line: cadainput6_2 = q_final;
cadaoutput6_1 = ADiGator_quatMTimes_2(cadainput6_1,cadainput6_2);
% Call to function: quatMTimes_2
qf_rot2goal.dx = cadaoutput6_1.dx; qf_rot2goal.f = cadaoutput6_1.f;
%User Line: qf_rot2goal     = cadaoutput6_1;
cadainput3_1.dx = qf_rot2goal.dx; cadainput3_1.f = qf_rot2goal.f;
%User Line: cadainput3_1 = qf_rot2goal;
[cadaoutput3_1,cadaoutput3_2] = ADiGator_quat2aa(cadainput3_1);
% Call to function: quat2aa
q_final_ax.dx = cadaoutput3_2.dx; q_final_ax.f = cadaoutput3_2.f;
%User Line: q_final_ax = cadaoutput3_2;
cadainput5_1.dx = -q_final_ax.dx;
cadainput5_1.f = Gator1Data.Data4 - q_final_ax.f;
%User Line: cadainput5_1 = [0 0 1]' - q_final_ax;
cadaoutput5_1 = ADiGator_norm_(cadainput5_1);
% Call to function: norm_
CON_E_q_final.dx = cadaoutput5_1.dx; CON_E_q_final.f = cadaoutput5_1.f;
%User Line: CON_E_q_final   = cadaoutput5_1;
%User Line: % goal
cada1f1 = para_qf.';
cada1td1 = zeros(4,90);
cada1td1(Gator1Data.Index37) = q.dx;
cada1td1 = cada1f1*cada1td1;
cada1td1 = cada1td1(:);
cada1f2dx = cada1td1(Gator1Data.Index38);
cada1f2 = cada1f1*q.f;
cada1tf1 = cada1f2(Gator1Data.Index39);
dist_2_goal.dx = -1./sqrt(1-cada1tf1(:).^2).*cada1f2dx;
dist_2_goal.f = acos(cada1f2);
%User Line: dist_2_goal = acos(para_qf'*q);
COST_GOAL.dx = dist_2_goal.dx;
COST_GOAL.f = sum(dist_2_goal.f);
%User Line: COST_GOAL = sum(dist_2_goal);
%User Line: % summary
cada1f1dx = para_cost_goal_k.*COST_GOAL.dx;
cada1f1 = para_cost_goal_k*COST_GOAL.f;
cada1f2dx = para_cost_tilt_k.*COST_tilt.dx;
cada1f2 = para_cost_tilt_k*COST_tilt.f;
cada1td1 = cada1f1dx;
cada1td1 = cada1td1 + cada1f2dx;
COST.dx = cada1td1;
COST.f = cada1f1 + cada1f2;
%User Line: COST  = para_cost_goal_k*COST_GOAL + para_cost_tilt_k*COST_tilt;
cada1td1 = zeros(264,1);
cada1td1(Gator1Data.Index40) = CON_L_obj_ang_bt_frame.dx;
cada1td1(Gator1Data.Index41) = CON_L_tilt.dx;
CON_L.dx = cada1td1;
CON_L.f = [CON_L_obj_ang_bt_frame.f;CON_L_tilt.f];
%User Line: CON_L = [CON_L_obj_ang_bt_frame; CON_L_tilt];
cada1td1 = zeros(6,1);
cada1td1(Gator1Data.Index42) = CON_E_q_init.dx;
cada1td1(Gator1Data.Index43) = CON_E_q_final.dx;
CON_E.dx = cada1td1;
CON_E.f = [CON_E_q_init.f;CON_E_q_final.f];
%User Line: CON_E = [CON_E_q_init; CON_E_q_final];
cada1td1 = zeros(360,1);
cada1td1(Gator1Data.Index44) = COST.dx;
cada1td1(Gator1Data.Index45) = CON_L.dx;
cada1td1(Gator1Data.Index46) = CON_E.dx;
FUN.dx = cada1td1;
FUN.f = [COST.f;CON_L.f;CON_E.f];
%User Line: FUN = [COST; CON_L; CON_E];
%User Line: return
%User Line: % Flow = [-inf; -inf(size(CON_L)); zeros(size(CON_E))];
%User Line: % Fupp = [inf; zeros(size(CON_L)); zeros(size(CON_E))];
%User Line: % xlow = -2*pi*ones(3*N, 1);
%User Line: % xupp = 2*pi*ones(3*N, 1);
%User Line: %
%User Line: % save snoptfiles/sizeInfo2.mat Flow Fupp xlow xupp
FUN.dx_size = [62,90];
FUN.dx_location = Gator1Data.Index47;
end
function mynorm = ADiGator_normByCol(M)
global ADiGator_FUNOPT2_ADiGatorJac
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.ADiGator_normByCol.Gator1Data;
% ADiGator Start Derivative Computations
cada1tf2 = M.f(Gator1Data.Index1);
cada1f1dx = 2.*cada1tf2(:).^(2-1).*M.dx;
cada1f1 = M.f.^2;
cada1f2 = 3;
cada1td1 = zeros(3,93);
cada1td1(Gator1Data.Index2) = cada1f1dx;
cada1td1 = sum(cada1td1,1);
cada1f3dx = cada1td1(:);
cada1f3 = sum(cada1f1,1);
cada1tf1 = cada1f3(Gator1Data.Index3);
mynorm.dx = (1/2)./sqrt(cada1tf1(:)).*cada1f3dx;
mynorm.dx(cada1tf1(:) == 0 & cada1f3dx == 0) = 0;
mynorm.f = sqrt(cada1f3);
%User Line: mynorm = (sqrt(sum((M).^2, 1)));
end
function [theta,n] = ADiGator_quat2aa(q)
global ADiGator_FUNOPT2_ADiGatorJac
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.ADiGator_quat2aa.Gator1Data;
% ADiGator Start Derivative Computations
cada1f1dx = q.dx(Gator1Data.Index1);
cada1f1 = q.f(1);
cada1f2dx = -1./sqrt(1-cada1f1.^2).*cada1f1dx;
cada1f2 = acos(cada1f1);
theta.dx = 2.*cada1f2dx;
theta.f = 2*cada1f2;
%User Line: theta = 2*acos(q(1));
qv.dx = q.dx(Gator1Data.Index3);
qv.f = q.f(Gator1Data.Index2,:);
%User Line: qv = q(2:4,:);
cadainput2_1.dx = qv.dx; cadainput2_1.f = qv.f;
%User Line: cadainput2_1 = qv;
cada1tempdx = cadainput2_1.dx;
cadainput2_1.dx = zeros(279,1);
cadainput2_1.dx(Gator1Data.Index4,1) = cada1tempdx;
cadainput2_1.f(3,30) = 0;
cadaoutput2_1 = ADiGator_normByCol(cadainput2_1);
% Call to function: normByCol
cadaoutput2_1.dx = cadaoutput2_1.dx(Gator1Data.Index5,1);
cadaoutput2_1.f = cadaoutput2_1.f(:,1:1);
cada1tempdx = cadaoutput2_1.dx(Gator1Data.Index6);
cada1tf1 = Gator1Data.Data1(Gator1Data.Index7);
qvnorm.dx = cada1tf1(:).*cada1tempdx;
qvnorm.f = Gator1Data.Data1*cadaoutput2_1.f;
%User Line: qvnorm = ones(3,1)*cadaoutput2_1;
cada1tf1 = qvnorm.f(Gator1Data.Index8);
cada1td1 = qv.dx./cada1tf1(:);
cada1tf1 = qv.f(Gator1Data.Index9);
cada1tf2 = qvnorm.f(Gator1Data.Index10);
cada1td1 = cada1td1 + -cada1tf1(:)./cada1tf2(:).^2.*qvnorm.dx;
n.dx = cada1td1;
n.f = qv.f./qvnorm.f;
%User Line: n = qv./qvnorm;
%User Line: % 	n = bsxfun(@rdivide, qv, qvnorm);
end
function qi = ADiGator_quatInv(q)
global ADiGator_FUNOPT2_ADiGatorJac
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.ADiGator_quatInv.Gator1Data;
% ADiGator Start Derivative Computations
A.f =  [1  0  0  0;		 0 -1  0  0;		 0  0 -1  0;		 0  0  0 -1];
%User Line: A = [1  0  0  0;		 0 -1  0  0;		 0  0 -1  0;		 0  0  0 -1];
cada1f1 = 4;
cada1td1 = zeros(4,90);
cada1td1(Gator1Data.Index1) = q.dx;
cada1td1 = A.f*cada1td1;
cada1td1 = cada1td1(:);
qi.dx = cada1td1(Gator1Data.Index2);
qi.f = A.f*q.f;
%User Line: qi = A*q;
%User Line: % q(2:4,:) = -q(2:4,:);
end
function n = ADiGator_norm_(x)
global ADiGator_FUNOPT2_ADiGatorJac
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.ADiGator_norm_.Gator1Data;
% ADiGator Start Derivative Computations
cada1f1dx = x.dx;
cada1f1 = x.f.';
cada1td2 = zeros(3,3);
cada1td2(Gator1Data.Index1) = cada1f1dx;
cada1td2 = x.f.'*cada1td2;
cada1td1 = cada1td2(Gator1Data.Index2);
cada1td1 = cada1td1(:);
cada1td2 = zeros(3,3);
cada1td2(Gator1Data.Index3) = x.dx;
cada1td2 = cada1f1*cada1td2;
cada1td2 = cada1td2(:);
cada1td1 = cada1td1 + cada1td2(Gator1Data.Index4);
cada1f2dx = cada1td1;
cada1f2 = cada1f1*x.f;
n.dx = (1/2)./sqrt(cada1f2).*cada1f2dx;
n.dx(cada1f2 == 0 & cada1f2dx == 0) = 0;
n.f = sqrt(cada1f2);
%User Line: n = sqrt(x'*x);
end
function qp = ADiGator_quatMTimes_2(q1,q2)
global ADiGator_FUNOPT2_ADiGatorJac
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.ADiGator_quatMTimes_2.Gator1Data;
% ADiGator Start Derivative Computations
s1.f = q1(1);
%User Line: s1 = q1(1);
v1.f = q1(Gator1Data.Index1);
%User Line: v1 = q1(2:4);
s2.dx = q2.dx(Gator1Data.Index2);
s2.f = q2.f(1);
%User Line: s2 = q2(1);
v2.dx = q2.dx(Gator1Data.Index4);
v2.f = q2.f(Gator1Data.Index3);
%User Line: v2 = q2(2:4);
%User Line: % cr_v = cross(v1, v2);
cada1f1 = v1.f(2);
cada1f2dx = v2.dx(Gator1Data.Index5);
cada1f2 = v2.f(3);
cada1f3 = cada1f1*cada1f2;
cada1f4 = v1.f(3);
cada1f5dx = v2.dx(Gator1Data.Index6);
cada1f5 = v2.f(2);
cada1f6 = cada1f4*cada1f5;
cada1f7 = cada1f3 - cada1f6;
cada1f8 = v1.f(3);
cada1f9dx = v2.dx(Gator1Data.Index7);
cada1f9 = v2.f(1);
cada1f10 = cada1f8*cada1f9;
cada1f11 = v1.f(1);
cada1f12dx = v2.dx(Gator1Data.Index8);
cada1f12 = v2.f(3);
cada1f13dx = cada1f11.*cada1f12dx;
cada1f13 = cada1f11*cada1f12;
cada1f14dx = -cada1f13dx;
cada1f14 = cada1f10 - cada1f13;
cada1f15 = v1.f(1);
cada1f16dx = v2.dx(Gator1Data.Index9);
cada1f16 = v2.f(2);
cada1f17dx = cada1f15.*cada1f16dx;
cada1f17 = cada1f15*cada1f16;
cada1f18 = v1.f(2);
cada1f19dx = v2.dx(Gator1Data.Index10);
cada1f19 = v2.f(1);
cada1f20 = cada1f18*cada1f19;
cada1f21dx = cada1f17dx;
cada1f21 = cada1f17 - cada1f20;
cada1td1 = zeros(6,1);
cada1td1(Gator1Data.Index11) = cada1f14dx;
cada1td1(Gator1Data.Index12) = cada1f21dx;
cr_v.dx = cada1td1;
cr_v.f = [cada1f7;cada1f14;cada1f21];
%User Line: cr_v =[v1(2)*v2(3) - v1(3)*v2(2);           v1(3)*v2(1) - v1(1)*v2(3);           v1(1)*v2(2) - v1(2)*v2(1)];
cada1f1 = s1.f*s2.f;
cada1f2dx = v2.dx;
cada1f2 = v2.f.';
cada1td1 = zeros(3,3);
cada1td1(Gator1Data.Index13) = cada1f2dx;
cada1td1 = v1.f.'*cada1td1;
cada1td1 = cada1td1(:);
cada1f3dx = cada1td1(Gator1Data.Index14);
cada1f3 = cada1f2*v1.f;
cada1f4dx = -cada1f3dx;
cada1f4 = cada1f1 - cada1f3;
cada1f5 = v2.f*s1.f;
cada1tempdx = s2.dx(Gator1Data.Index15);
cada1tf1 = v1.f(Gator1Data.Index17);
cada1f6dx = cada1tf1(:).*cada1tempdx(Gator1Data.Index16);
cada1f6 = s2.f*v1.f;
cada1f7dx = cada1f6dx;
cada1f7 = cada1f5 + cada1f6;
cada1td1 = zeros(9,1);
cada1td1(Gator1Data.Index18) = cada1f7dx;
cada1td1(Gator1Data.Index19) = cada1td1(Gator1Data.Index19) + cr_v.dx;
cada1f8dx = cada1td1;
cada1f8 = cada1f7 + cr_v.f;
cada1td1 = zeros(12,1);
cada1td1(Gator1Data.Index20) = cada1f4dx;
cada1td1(Gator1Data.Index21) = cada1f8dx;
qp.dx = cada1td1;
qp.f = [cada1f4;cada1f8];
%User Line: qp = [s1*s2 - v2'*v1; v2*s1 + s2*v1 + cr_v];
end
function qp = ADiGator_quatMTimes_N1(q1,q2)
global ADiGator_FUNOPT2_ADiGatorJac
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.ADiGator_quatMTimes_N1.Gator1Data;
% ADiGator Start Derivative Computations
s1.dx = q1.dx(Gator1Data.Index1);
s1.f = q1.f(1,:);
%User Line: s1 = q1(1,:);
v1.dx = q1.dx(Gator1Data.Index2);
v1.f = q1.f(Gator1Data.Index9,:);
%User Line: v1 = q1(2:4,:);
s2.f = q2.f(1);
%User Line: s2 = q2(1);
v2.f = q2.f(Gator1Data.Index10);
%User Line: v2 = q2(2:4);
%User Line: % cr_v = cross(v1, v2);
cada1f1dx = v1.dx(Gator1Data.Index3);
cada1f1 = v1.f(2,:);
cada1f2 = v2.f(3);
cada1f4dx = cada1f2.*cada1f1dx;
cada1f4 = cada1f1*cada1f2;
cada1f5dx = v1.dx(Gator1Data.Index4);
cada1f5 = v1.f(3,:);
cada1f6 = v2.f(2);
cada1f8dx = cada1f6.*cada1f5dx;
cada1f8 = cada1f5*cada1f6;
cada1td1 = cada1f4dx;
cada1td1 = cada1td1 + -cada1f8dx;
cada1f9dx = cada1td1;
cada1f9 = cada1f4 - cada1f8;
cada1f10dx = v1.dx(Gator1Data.Index5);
cada1f10 = v1.f(3,:);
cada1f11 = v2.f(1);
cada1f13dx = cada1f11.*cada1f10dx;
cada1f13 = cada1f10*cada1f11;
cada1f14dx = v1.dx(Gator1Data.Index6);
cada1f14 = v1.f(1,:);
cada1f15 = v2.f(3);
cada1f17dx = cada1f15.*cada1f14dx;
cada1f17 = cada1f14*cada1f15;
cada1td1 = cada1f13dx;
cada1td1 = cada1td1 + -cada1f17dx;
cada1f18dx = cada1td1;
cada1f18 = cada1f13 - cada1f17;
cada1f19dx = v1.dx(Gator1Data.Index7);
cada1f19 = v1.f(1,:);
cada1f20 = v2.f(2);
cada1f22dx = cada1f20.*cada1f19dx;
cada1f22 = cada1f19*cada1f20;
cada1f23dx = v1.dx(Gator1Data.Index8);
cada1f23 = v1.f(2,:);
cada1f24 = v2.f(1);
cada1f26dx = cada1f24.*cada1f23dx;
cada1f26 = cada1f23*cada1f24;
cada1td1 = cada1f22dx;
cada1td1 = cada1td1 + -cada1f26dx;
cada1f27dx = cada1td1;
cada1f27 = cada1f22 - cada1f26;
cada1td1 = zeros(270,1);
cada1td1(Gator1Data.Index11) = cada1f9dx;
cada1td1(Gator1Data.Index12) = cada1f18dx;
cada1td1(Gator1Data.Index13) = cada1f27dx;
cr_v.dx = cada1td1;
cr_v.f = [cada1f9;cada1f18;cada1f27];
%User Line: cr_v =[v1(2,:)*v2(3) - v1(3,:)*v2(2);           v1(3,:)*v2(1) - v1(1,:)*v2(3);           v1(1,:)*v2(2) - v1(2,:)*v2(1)];
cada1f2 = s1.f*s2.f;
cada1f3 = v2.f.';
cada1f4 = 3;
cada1td1 = zeros(3,90);
cada1td1(Gator1Data.Index14) = v1.dx;
cada1td1 = cada1f3*cada1td1;
cada1td1 = cada1td1(:);
cada1f5dx = cada1td1(Gator1Data.Index15);
cada1f5 = cada1f3*v1.f;
cada1f6dx = -cada1f5dx;
cada1f6 = cada1f2 - cada1f5;
cada1f7 = 1;
cada1td1 = zeros(1,90);
cada1td1(Gator1Data.Index16) = s1.dx;
cada1td1 = v2.f*cada1td1;
cada1td1 = cada1td1(:);
cada1f8dx = cada1td1(Gator1Data.Index17);
cada1f8 = v2.f*s1.f;
cada1f10 = s2.f*v1.f;
cada1f11dx = cada1f8dx;
cada1f11 = cada1f8 + cada1f10;
cada1td1 = cada1f11dx;
cada1td1 = cada1td1 + cr_v.dx;
cada1f12dx = cada1td1;
cada1f12 = cada1f11 + cr_v.f;
cada1td1 = zeros(360,1);
cada1td1(Gator1Data.Index18) = cada1f6dx;
cada1td1(Gator1Data.Index19) = cada1f12dx;
qp.dx = cada1td1;
qp.f = [cada1f6;cada1f12];
%User Line: qp = [s1*s2 - v2'*v1; v2*s1 + s2*v1 + cr_v];
end
function qpv = ADiGator_quatMTimes_NN(q1,q2)
global ADiGator_FUNOPT2_ADiGatorJac
Gator1Data = ADiGator_FUNOPT2_ADiGatorJac.ADiGator_quatMTimes_NN.Gator1Data;
% ADiGator Start Derivative Computations
s1.dx = q1.dx(Gator1Data.Index1);
s1.f = q1.f(1,:);
%User Line: s1 = q1(1,:);
v1.dx = q1.dx(Gator1Data.Index2);
v1.f = q1.f(Gator1Data.Index17,:);
%User Line: v1 = q1(2:4,:);
s2.dx = q2.dx(Gator1Data.Index3);
s2.f = q2.f(1,:);
%User Line: s2 = q2(1,:);
v2.dx = q2.dx(Gator1Data.Index4);
v2.f = q2.f(Gator1Data.Index18,:);
%User Line: v2 = q2(2:4,:);
%User Line: % cr_v = cross(v1, v2);
cada1f1dx = v1.dx(Gator1Data.Index5);
cada1f1 = v1.f(2,:);
cada1f2dx = v2.dx(Gator1Data.Index6);
cada1f2 = v2.f(3,:);
cada1tf1 = cada1f2(Gator1Data.Index19);
cada1td1 = cada1tf1(:).*cada1f1dx;
cada1tf1 = cada1f1(Gator1Data.Index20);
cada1td1 = cada1td1 + cada1tf1(:).*cada1f2dx;
cada1f3dx = cada1td1;
cada1f3 = cada1f1.*cada1f2;
cada1f4dx = v1.dx(Gator1Data.Index7);
cada1f4 = v1.f(3,:);
cada1f5dx = v2.dx(Gator1Data.Index8);
cada1f5 = v2.f(2,:);
cada1tf1 = cada1f5(Gator1Data.Index21);
cada1td1 = cada1tf1(:).*cada1f4dx;
cada1tf1 = cada1f4(Gator1Data.Index22);
cada1td1 = cada1td1 + cada1tf1(:).*cada1f5dx;
cada1f6dx = cada1td1;
cada1f6 = cada1f4.*cada1f5;
cada1td1 = cada1f3dx;
cada1td1 = cada1td1 + -cada1f6dx;
cada1f7dx = cada1td1;
cada1f7 = cada1f3 - cada1f6;
cada1f8dx = v1.dx(Gator1Data.Index9);
cada1f8 = v1.f(3,:);
cada1f9dx = v2.dx(Gator1Data.Index10);
cada1f9 = v2.f(1,:);
cada1tf1 = cada1f9(Gator1Data.Index23);
cada1td1 = cada1tf1(:).*cada1f8dx;
cada1tf1 = cada1f8(Gator1Data.Index24);
cada1td1 = cada1td1 + cada1tf1(:).*cada1f9dx;
cada1f10dx = cada1td1;
cada1f10 = cada1f8.*cada1f9;
cada1f11dx = v1.dx(Gator1Data.Index11);
cada1f11 = v1.f(1,:);
cada1f12dx = v2.dx(Gator1Data.Index12);
cada1f12 = v2.f(3,:);
cada1tf1 = cada1f12(Gator1Data.Index25);
cada1td1 = cada1tf1(:).*cada1f11dx;
cada1tf1 = cada1f11(Gator1Data.Index26);
cada1td1 = cada1td1 + cada1tf1(:).*cada1f12dx;
cada1f13dx = cada1td1;
cada1f13 = cada1f11.*cada1f12;
cada1td1 = cada1f10dx;
cada1td1 = cada1td1 + -cada1f13dx;
cada1f14dx = cada1td1;
cada1f14 = cada1f10 - cada1f13;
cada1f15dx = v1.dx(Gator1Data.Index13);
cada1f15 = v1.f(1,:);
cada1f16dx = v2.dx(Gator1Data.Index14);
cada1f16 = v2.f(2,:);
cada1tf1 = cada1f16(Gator1Data.Index27);
cada1td1 = cada1tf1(:).*cada1f15dx;
cada1tf1 = cada1f15(Gator1Data.Index28);
cada1td1 = cada1td1 + cada1tf1(:).*cada1f16dx;
cada1f17dx = cada1td1;
cada1f17 = cada1f15.*cada1f16;
cada1f18dx = v1.dx(Gator1Data.Index15);
cada1f18 = v1.f(2,:);
cada1f19dx = v2.dx(Gator1Data.Index16);
cada1f19 = v2.f(1,:);
cada1tf1 = cada1f19(Gator1Data.Index29);
cada1td1 = cada1tf1(:).*cada1f18dx;
cada1tf1 = cada1f18(Gator1Data.Index30);
cada1td1 = cada1td1 + cada1tf1(:).*cada1f19dx;
cada1f20dx = cada1td1;
cada1f20 = cada1f18.*cada1f19;
cada1td1 = cada1f17dx;
cada1td1 = cada1td1 + -cada1f20dx;
cada1f21dx = cada1td1;
cada1f21 = cada1f17 - cada1f20;
cada1td1 = zeros(270,1);
cada1td1(Gator1Data.Index31) = cada1f7dx;
cada1td1(Gator1Data.Index32) = cada1f14dx;
cada1td1(Gator1Data.Index33) = cada1f21dx;
cr_v.dx = cada1td1;
cr_v.f = [cada1f7;cada1f14;cada1f21];
%User Line: cr_v =[v1(2,:).*v2(3,:) - v1(3,:).*v2(2,:);           v1(3,:).*v2(1,:) - v1(1,:).*v2(3,:);           v1(1,:).*v2(2,:) - v1(2,:).*v2(1,:)];
%User Line: % qp = [s1.*s2 - sum(v2.*v1); bsxfun(@times, v2, s1) + bsxfun(@times, s2, v1) + cr_v];
cada1f1 = 1;
cada1td1 = zeros(1,90);
cada1td1(Gator1Data.Index34) = s1.dx;
cada1td1 = Gator1Data.Data1*cada1td1;
cada1td1 = cada1td1(:);
s1.dx = cada1td1(Gator1Data.Index35);
s1.f = Gator1Data.Data1*s1.f;
%User Line: s1 = ones(3,1)*s1;
cada1f1 = 1;
cada1td1 = zeros(1,90);
cada1td1(Gator1Data.Index36) = s2.dx;
cada1td1 = Gator1Data.Data2*cada1td1;
cada1td1 = cada1td1(:);
s2.dx = cada1td1(Gator1Data.Index37);
s2.f = Gator1Data.Data2*s2.f;
%User Line: s2 = ones(3,1)*s2;
cada1tf1 = v2.f(Gator1Data.Index38);
cada1td1 = cada1tf1(:).*s1.dx;
cada1tf1 = s1.f(Gator1Data.Index39);
cada1td1 = cada1td1 + cada1tf1(:).*v2.dx;
cada1f1dx = cada1td1;
cada1f1 = s1.f.*v2.f;
cada1tf1 = v1.f(Gator1Data.Index40);
cada1td1 = cada1tf1(:).*s2.dx;
cada1tf1 = s2.f(Gator1Data.Index41);
cada1td1 = cada1td1 + cada1tf1(:).*v1.dx;
cada1f2dx = cada1td1;
cada1f2 = s2.f.*v1.f;
cada1td1 = cada1f1dx;
cada1td1 = cada1td1 + cada1f2dx;
cada1f3dx = cada1td1;
cada1f3 = cada1f1 + cada1f2;
cada1td1 = cada1f3dx;
cada1td1 = cada1td1 + cr_v.dx;
qpv.dx = cada1td1;
qpv.f = cada1f3 + cr_v.f;
%User Line: qpv = s1.*v2 + s2.*v1 + cr_v;
%User Line: % qpv = bsxfun(@times, v2, s1) + bsxfun(@times, s2, v1) + cr_v;
end


function ADiGator_LoadData()
global ADiGator_FUNOPT2_ADiGatorJac
ADiGator_FUNOPT2_ADiGatorJac = load('FUNOPT2_ADiGatorJac.mat');
return
end