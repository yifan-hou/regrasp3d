% Quadratic optimization
% -------------------------------------------
%   constant computation
% 	min x'Qx
% 	s.t. Ax=B
% 		 xlow < x < xupp
% -------------------------------------------

function [Q, A, B, Flow, Fupp, xlow, xupp] = FUNOPT_GRP_Precomputation(x, para)

N = para.N;
para_obj_rotation = para.obj_rotation;
para_rtype        = para.rtype; % 1: pivoting
para_range        = para.range;
para_x0           = para.x0;
para_xf           = para.xf;
para_x0_k         = para.x0_k;
para_xf_k         = para.xf_k;
para_Qreg         = para.Qreg;

%
% Range: XL, XU
%

% initial/final grasps
para_range(1,1) = para_range(1,1)*(1-para_x0_k) + para_x0*para_x0_k;
para_range(2,1) = para_range(2,1)*(1-para_x0_k) + para_x0*para_x0_k;
para_range(1,N) = para_range(1,N)*(1-para_xf_k) + para_xf*para_xf_k;
para_range(2,N) = para_range(2,N)*(1-para_xf_k) + para_xf*para_xf_k;


%
% rtype
%
Rt = diag(1 - para_rtype(1:end-1));
U = [zeros(N-1,1) diag(ones(N-1, 1))];
L = [diag(ones(N-1,1)) zeros(N-1, 1)];
b = - para_obj_rotation(2:end)' + para_obj_rotation(1:end-1)';
A = Rt*(U-L);
B = Rt*b;

%
% cost
%
Qdiag          = ones(1, N);
Qdiag(2:end-1) = 2;
Qdiag1         = -ones(1, N-1);
Q              = diag(Qdiag) + diag(Qdiag1, 1) + diag(Qdiag1, -1);
Q = Q + eye(N)*para_Qreg;

Flow = [-inf; B];
Fupp = [ inf; B];
xlow = para_range(1, :)';
xupp = para_range(2, :)';


end