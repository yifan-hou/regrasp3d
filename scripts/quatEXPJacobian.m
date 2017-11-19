function q = quatEXPJacobian(in1)
%QUATEXPJACOBIAN
%    Q = QUATEXPJACOBIAN(IN1)
%    Derived by:
%       n = sym('n', [3, 1], 'real');
%       theta = norm(n);
%       q = [cos(theta); sin(theta)*n/theta];
%       q = simplify(q);
%       G = [diff(q, 'n1') diff(q, 'n2') diff(q, 'n3')];
%       q_ = matlabFunction(q, 'File', 'quatEXPJacobian', 'vars',{n});
% 
%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    16-Nov-2017 12:09:38

n1 = in1(1,:);
n2 = in1(2,:);
n3 = in1(3,:);
t2 = n1.^2;
t3 = n2.^2;
t4 = n3.^2;
t5 = t2+t3+t4;
t6 = sqrt(t5);
t7 = sin(t6);
t8 = 1.0./sqrt(t5);
q = [cos(t6);n1.*t7.*t8;n2.*t7.*t8;n3.*t7.*t8];
