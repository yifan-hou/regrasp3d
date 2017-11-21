clear;clc;

addpath autodiff_generated
addpath snoptfiles

N = 30;

global paraOpt2
paraOpt2.delta_theta   = 0.05;
paraOpt2.q0            = [1 0 0 0]';
paraOpt2.qf            = [0 1 0 0]';
paraOpt2.qf_inv        = quatInv(paraOpt2.qf);
paraOpt2.Gp1o          = [0.8 0.5 0.4]';
paraOpt2.Gp2o          = [0.8 0.6 0.4]';
% paraOpt2.GpZ_limit     = 0.1;
paraOpt2.Gp_tilt_limit = 50*pi/180;
paraOpt2.cost_goal_k = 10;
paraOpt2.cost_tilt_k = 1;

qi = quatSlerp(paraOpt2.q0, paraOpt2.qf, [0:0.03:1]);
x  = quatLog(qi);
x  = x(2:4, :);
x  = x(:);

% tic;
% for i = 1:100
%     x = rand(3*N,1);
%     FUNOPT2(x, paraOpt2);
% end
% toc;
% return;

FUNOPT2(x, paraOpt2);
load snoptfiles/sizeInfo2.mat

snprint('snoptfiles/snoptOPT2.out');
snoptOPT2.spc = which('snoptfiles/snoptOPT2.spc');
snspec ( snoptOPT2.spc );

snseti ('Major Iteration limit', 50);
snset  ('Minimize');
tic
[x,F,INFO] = snopt(x,xlow,xupp,Flow,Fupp,'FUNOPT2Wrapper');
toc
% itns   =  sngeti ('iw 421')
% majors =  sngeti ('iw 422')

snprint off; % Closes the file and empties the print buffer



% ---------------------------------------------
disp('Results:');
n = zeros(3, N);
for i = 1:N
	n(1, i) = x((i-1)*3+1);
	n(2, i) = x((i-1)*3+2);
	n(3, i) = x((i-1)*3+3);
end
theta                  = normByCol(n);
temp                   = ones(3,1)*(sin(theta)./theta);
q                      = [cos(theta); temp.*n];

for i = 1:N
	disp(q(:,i)');
end