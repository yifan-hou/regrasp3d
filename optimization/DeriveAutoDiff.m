clear;clc;
addpath autodiff/
addpath autodiff_generated/

% ---------------------------- Size information ----------------------
N = 30;
x = rand(3*N,1);

para.delta_theta   = 0.05;
para.q0            = [1 0 0 0]';
para.qf            = [0 1 0 0]';
para.qf_inv        = quatInv(para.qf);
para.Gp1o          = [0.8 0.5 0.4]';
para.Gp2o          = [0.8 0.6 0.4]';
% para.GpZ_limit     = 0.1;
para.Gp_tilt_limit = 50*pi/180;
para.cost_goal_k = 10;
para.cost_tilt_k = 1;



% ------------------------------ ADiGator ------------------------------- %
func_name = 'FUNOPT1';
gx = adigatorCreateDerivInput([3*N, 1],'x'); % Create Deriv Input
genout = adigatorGenJacFile(func_name,{gx, para});
S = genout.JacobianStructure;

% move files
movefile FUNOPT1_* autodiff_generated/

disp('[Evaluation]');
numeval = 10;
tic;
for i = 1:numeval
	x = rand(3*N,1);
	[Jac, y] = FUNOPT1_Jac(x, para);
end
adigatortime = toc/numeval;
display(['Average deriv eval time using ADiGator:                     ',num2str(adigatortime)]);




func_name = 'FUNOPT2';
gx = adigatorCreateDerivInput([3*N, 1],'x'); % Create Deriv Input
genout = adigatorGenJacFile(func_name,{gx, para});
S = genout.JacobianStructure;

% move files
movefile FUNOPT2_* autodiff_generated/

disp('[Evaluation]');
numeval = 10;
tic;
for i = 1:numeval
	x = rand(3*N,1);
	[Jac, y] = FUNOPT2_Jac(x, para);
end
adigatortime = toc/numeval;
display(['Average deriv eval time using ADiGator:                     ',num2str(adigatortime)]);

