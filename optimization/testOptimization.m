clear;clc;

addpath autodiff_generated
addpath snoptfiles
addpath autodiff

% % ----------------------------------------------------
% % 		Test FUNOPT_OBJ
% % ----------------------------------------------------
% 
% N = 20;
% 
% global paraOpt_OBJ
% paraOpt_OBJ.delta_theta   = 0.2;
% paraOpt_OBJ.q0            = [1 0 0 0]';
% paraOpt_OBJ.qf            = [0 1 0 0]';
% paraOpt_OBJ.qf_inv        = quatInv(paraOpt_OBJ.qf);
% paraOpt_OBJ.Gp1o          = [0.7 0.6 0.4]';
% paraOpt_OBJ.Gp2o          = [0.8 0.6 0.4]';
% paraOpt_OBJ.Gp_tilt_limit = 50*pi/180;
% paraOpt_OBJ.cost_dq_k            = 2;
% paraOpt_OBJ.cost_tilt_k   = 1;
% paraOpt_OBJ.exactqf_k     = 1;
% 
% x = rand(3*N,1);
% x(1:3) = 0;
% 
% [FUN, Flow, Fupp, xlow, xupp] = FUNOPT_OBJ(x, paraOpt_OBJ);
% 
% snprint('snoptfiles/snoptOPT_OBJ.out');
% snoptOPT.spc = which('snoptfiles/snoptOPT_OBJ.spc');
% snspec ( snoptOPT.spc );
% 
% snseti ('Major Iteration limit', 30);
% snset  ('Minimize');
% 
% tic
% [x, F, INFO] = snopt(x,xlow,xupp,Flow,Fupp,'FUNOPT_OBJWrapper');
% toc
% % itns   =  sngeti ('iw 421')
% % majors =  sngeti ('iw 422')
% 
% snprint off; % Closes the file and empties the print buffer
% 
% % ---------------------------------------------
% % check constraints
% f = FUNOPT_OBJ(x, paraOpt_OBJ);
% cost = f(1);
% con = zeros(length(f), 2);
% for i = 1:length(f)
% 	if f(i) < Flow(i)
% 		con(i, 1) = Flow(i) - f(i);
% 	end
% 
% 	if f(i) > Fupp(i)
% 		con(i, 2) = f(i) - Fupp(i);
% 	end
% end
% cost
% con
% 
% 
% % ---------------------------------------------
% disp('Results:');
% n = zeros(3, N);
% for i = 1:N
% 	n(1, i) = x((i-1)*3+1);
% 	n(2, i) = x((i-1)*3+2);
% 	n(3, i) = x((i-1)*3+3);
% end
% theta                  = normByCol(n);
% id                     = theta < 1e-8;
% temp                   = ones(3,1)*(sin(theta/2)./theta);
% temp(:, id)            = 0;
% q                      = [cos(theta/2); temp.*n];
% 
% for i = 1:N
% 	disp(q(:,i)');
% end



% ----------------------------------------------------
% 		Test FUNOPT_GRP
% ----------------------------------------------------
global paraOpt_GRP
N = 100;
x = zeros(N,1);

xrange                  = zeros(2, N);
xrange(1,:)             = sin((1:N)*0.1)-0.1;
xrange(2,:)             = sin((1:N)*0.1-0.5)+0.8;
paraOpt_GRP.range        = xrange;
paraOpt_GRP.x0_k         = 1;
paraOpt_GRP.xf_k         = 0;
paraOpt_GRP.N            = N;
paraOpt_GRP.rtype        = round(zeros(1, N));
paraOpt_GRP.obj_rotation = rand(1, N)/5;
paraOpt_GRP.x0           = -0.4;
paraOpt_GRP.xf           = 0.6;
paraOpt_GRP.Qreg		 = 1e-3;

[Q, A, B, Flow, Fupp, xlow, xupp] = FUNOPT_GRP_Precomputation(x, paraOpt_GRP);
[iAfun, jAvar, A] = find([zeros(size(x'*Q)); A]);
[iGfun, jGvar]    = find(ones(size(Q*x)));
global paraQ
paraQ = Q;

snprint('snoptfiles/snoptOPT_GRP.out');
snoptOPT.spc = which('snoptfiles/snoptOPT_GRP.spc');
snspec ( snoptOPT.spc );

snseti ('Major Iteration limit', 50);
snset  ('Minimize');

tic
[x, F, INFO] = snopt(x,xlow,xupp,Flow,Fupp,'FUNOPT_GRP', ...
		   A, iAfun, jAvar, iGfun, jGvar);
toc
% itns   =  sngeti ('iw 421')
% majors =  sngeti ('iw 422')

snprint off; % Closes the file and empties the print buffer

% ---------------------------------------------
% check constraints
figure(1); clf(1);hold on;
plot([1:N], x,'- .b');
plot(find(paraOpt_GRP.rtype~=0), x(paraOpt_GRP.rtype~=0),'og');
plot([1:N], xrange(1,:), '-r');
plot([1:N], xrange(2,:), '-r');
plot(1, paraOpt_GRP.x0, '.k', 'markersize',10);
plot(N, paraOpt_GRP.xf, '.k', 'markersize',10);