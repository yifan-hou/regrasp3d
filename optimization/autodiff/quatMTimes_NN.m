% quaternion multiplication
% rotate by p then q, is equal to rotate by qp.
% input:
% 	q1: 4xN cuda  	q2: 4xN cuda
% output:
%	qp: 4x1
function qpv = quatMTimes_NN(q1, q2)
	s1 = q1(1,:);
	v1 = q1(2:4,:);

	s2 = q2(1,:);
	v2 = q2(2:4,:);

    % cr_v = cross(v1, v2);
    cr_v =[v1(2,:).*v2(3,:) - v1(3,:).*v2(2,:);
           v1(3,:).*v2(1,:) - v1(1,:).*v2(3,:);
           v1(1,:).*v2(2,:) - v1(2,:).*v2(1,:)];

    % qp = [s1.*s2 - sum(v2.*v1); bsxfun(@times, v2, s1) + bsxfun(@times, s2, v1) + cr_v];
    s1 = ones(3,1)*s1;
    s2 = ones(3,1)*s2;
    qpv = s1.*v2 + s2.*v1 + cr_v;
    % qpv = bsxfun(@times, v2, s1) + bsxfun(@times, s2, v1) + cr_v;
end



% function c = cross_(a, b)
%     c =[a(2)*b(3) - a(3)*b(2);
%         a(3)*b(1) - a(1)*b(3);
%         a(1)*b(2) - a(2)*b(1)];
% end
