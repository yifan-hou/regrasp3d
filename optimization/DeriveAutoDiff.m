N = 30;
x = rand(3*N,1);

para.para_delta_theta   = 0.05;
para.para_q0            = [1 0 0 0]';
para.para_qf            = [0 1 0 0]';
para.para_Gp1o          = [0.8 0.5 0.4]';
para.para_Gp2o          = [0.8 0.6 0.4]';
% para.para_GpZ_limit     = 0.1;
para.para_Gp_tilt_limit = 50*pi/180;
para.para_cost_goal_k = 10;
para.para_cost_tilt_k = 1;



% ------------------------------ ADiGator ------------------------------- %
gx = adigatorCreateDerivInput([3*N, 1],'x'); % Create Deriv Input
genout = adigatorGenJacFile('FUNOPT2',{gx, para});
S = genout.JacobianStructure;

disp('[Evaluation]');
numeval = 10;
tic;
for i = 1:numeval
  [Jac, y] = FUNOPT2(x);
end
adigatortime = toc/numeval;


% -------------------------- Finite Differences ------------------------- %
TOL = 1e-8;% Can change this to make numjac more/less accurate
tic
for i = 1:numeval
  dfdx = numjac(@FUNOPT24numjac,0,x,y,TOL*ones(3*N,1),[],0);
end
fdtime = toc/numeval;

display(['Average deriv eval time using Finite Differences:           ',num2str(fdtime)]);
display(['Average deriv eval time using ADiGator:                     ',num2str(adigatortime)]);