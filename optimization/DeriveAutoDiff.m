clear;clc;
addpath autodiff/
addpath autodiff_generated/

% % ---------------------------- FUNOPT_OBJ ----------------------
% disp('Deriving for FUNOPT_OBJ.');
% N = 20;
% x = rand(3*N,1);
% 
% para.delta_theta   = 0.15;
% para.q0            = [1 0 0 0]';
% para.qf            = [0 1 0 0]';
% para.qf_inv        = quatInv(para.qf);
% para.Gp1o          = [0.8 0.5 0.4]';
% para.Gp2o          = [0.8 0.6 0.4]';
% para.Gp_tilt_limit = 50*pi/180;
% para.exactqf_k     = 1;
% para.cost_dq_k     = 2;
% para.cost_tilt_k   = 1;
% 
% func_name = 'FUNOPT_OBJ';
% gx = adigatorCreateDerivInput([3*N, 1],'x'); % Create Deriv Input
% genout = adigatorGenJacFile(func_name,{gx, para});
% S = genout.JacobianStructure;
% 
% % move files
% movefile FUNOPT_OBJ_* autodiff_generated/
% 
% disp('[Evaluation]');
% numeval = 20;
% tic;
% for i = 1:numeval
% 	x = rand(3*N,1);
% 	[Jac, y] = FUNOPT_OBJ_Jac(x, para);
% end
% adigatortime = toc/numeval;
% display(['Average deriv eval time using ADiGator:                     ',num2str(adigatortime)]);








% ---------------------------- FUNOPT_GRP ----------------------
disp('Deriving for FUNOPT_GRP.');
N = 100;
x = rand(N,1);

xrange                  = zeros(2, N);
xrange(1,:)             = ones(1, N) + rand(1, N);
xrange(2,:)             = rand(1, N);
paraOptGRP.range        = xrange;
paraOptGRP.x0_k         = 1;
paraOptGRP.xf_k         = 0;
paraOptGRP.N            = N;
paraOptGRP.rtype        = round(rand(1, N));
paraOptGRP.obj_rotation = rand(1, N)/5;
paraOptGRP.x0           = -0.5;
paraOptGRP.xf           = 0.6;

func_name = 'FUNOPT_GRP';
gx = adigatorCreateDerivInput([N, 1],'x'); % Create Deriv Input
genout = adigatorGenJacFile(func_name,{gx, paraOptGRP});
S = genout.JacobianStructure;

% move files
movefile FUNOPT_GRP_* autodiff_generated/

disp('[Evaluation]');
numeval = 20;
tic;
for i = 1:numeval
	x = rand(N,1);
	[Jac, y] = FUNOPT_GRP_Jac(x, paraOptGRP);
end
adigatortime = toc/numeval;
display(['Average deriv eval time using ADiGator:                     ',num2str(adigatortime)]);

