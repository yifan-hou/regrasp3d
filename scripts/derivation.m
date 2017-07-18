clear;clc;
% symbols
% q0     = sym('q0','real');
q1 = sym('q1','real');
q2 = sym('q2','real');
q3 = sym('q3','real');

v0 = sym('v0',[3,1],'real'); 

% 
q0 = sqrt(1 - q1^2 - q2^2 - q3^2);
% 
A3 = [2*(q1*q3-q2*q0) 2*(q2*q3+q1*q0) 1-2*q1^2-2*q2^2];

z = A3*v0;
F = z^2;

% % Gradient
% grad_q   = @(f) [diff(f,'q0'); diff(f,'q1'); diff(f,'q2'); diff(f,'q3')];
% deriv_q  = @(f) [diff(f,'q0')  diff(f,'q1')  diff(f,'q2')  diff(f,'q3')];
grad_q   = @(f) [diff(f,'q1'); diff(f,'q2'); diff(f,'q3')];
deriv_q  = @(f) [diff(f,'q1')  diff(f,'q2')  diff(f,'q3')];

% 
Fq  = simplify(grad_q(F));
Fqq = simplify(deriv_q(Fq));


%% ---------------------------------------------------------------
% 			write to file
% ---------------------------------------------------------------
% f_  = matlabFunction(f, 'File', 'generated/f_', 'vars',{x,u,T, D_inv});
