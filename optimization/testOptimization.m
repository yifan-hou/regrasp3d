clear;clc;

addpath autodiff_generated
addpath snoptfiles
addpath autodiff

N = 30;

global paraOpt
paraOpt.delta_theta   = 0.1;
paraOpt.q0            = [1 0 0 0]';
paraOpt.qf            = [0 1 0 0]';
paraOpt.qf_inv        = quatInv(paraOpt.qf);
paraOpt.Gp1o          = [0.8 0.5 0.4]';
paraOpt.Gp2o          = [0.8 0.6 0.4]';
% paraOpt.GpZ_limit     = 0.1;
paraOpt.Gp_tilt_limit = 50*pi/180;
paraOpt.cost_goal_k = 10;
paraOpt.cost_tilt_k = 1;

x = rand(3*N,1);

% tic;
% for i = 1:100
%     x = rand(3*N,1);
%     FUNOPT2(x, paraOpt);
% end
% toc;
% return;

FUNOPT1(x, paraOpt);
load snoptfiles/sizeInfo1.mat

snprint('snoptfiles/snoptOPT1.out');
snoptOPT.spc = which('snoptfiles/snoptOPT.spc');
snspec ( snoptOPT.spc );

snseti ('Major Iteration limit', 50);
snset  ('Minimize');
tic
[x,F,INFO] = snopt(x,xlow,xupp,Flow,Fupp,'FUNOPT1Wrapper');
toc
% itns   =  sngeti ('iw 421')
% majors =  sngeti ('iw 422')

snprint off; % Closes the file and empties the print buffer

% ---------------------------------------------
% check constraints
f = FUNOPT1(x, paraOpt);
cost = f(1);
con = zeros(length(f), 2);
for i = 1:length(f)
	if f(i) < Flow(i)
		con(i, 1) = Flow(i) - f(i);
	end

	if f(i) > Fupp(i)
		con(i, 2) = f(i) - Fupp(i);
	end
end
cost
con


% ---------------------------------------------
disp('Results:');
n = zeros(3, N);
for i = 1:N
	n(1, i) = x((i-1)*3+1);
	n(2, i) = x((i-1)*3+2);
	n(3, i) = x((i-1)*3+3);
end
theta                  = normByCol(n);
id                     = theta < 1e-8;
temp                   = ones(3,1)*(sin(theta)./theta);
temp(:, id)            = 0;
q                      = [cos(theta); temp.*n];

for i = 1:N
	disp(q(:,i)');
end