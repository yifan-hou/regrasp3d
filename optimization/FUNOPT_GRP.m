% pose to pose

% Quadratic optimization
% No need for adigator
function [f, g] = FUNOPT_GRP(x)
global paraQ

% -------------------------------------------
% 	min x'Qx
% 	s.t. Ax=B
% 		 xlow < x < xupp
% -------------------------------------------

COST = x'*paraQ*x;
N    = length(x);
f    = [COST; zeros(N-1, 1)]; % snopt computes the linear entries

if nargout == 1
	return;
end

g = 2*paraQ*x; % array of nonlinear entries of jacobian
% snopt computes the linear entries

end