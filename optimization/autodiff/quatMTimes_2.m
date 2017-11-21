% quaternion multiplication
% rotate by p then q, is equal to rotate by qp.
% input:
% 	q1: 4x1 number  	q2: 4x1 cuda
% output:
%	qp: 4x1
function qp = quatMTimes_2(q1, q2)
	s1 = q1(1);
	v1 = q1(2:4);

	s2 = q2(1);
	v2 = q2(2:4);

    % cr_v = cross(v1, v2);

    cr_v =[v1(2)*v2(3) - v1(3)*v2(2);
           v1(3)*v2(1) - v1(1)*v2(3);
           v1(1)*v2(2) - v1(2)*v2(1)];
    qp = [s1*s2 - v2'*v1; v2*s1 + s2*v1 + cr_v];
end
