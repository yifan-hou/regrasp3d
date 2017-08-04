% q moves [1 0 0] to (gp1 - gp2), [0 1 0] to [? ? 0]
function q = getProperGrasp(gp1, gp2)
v  = gp1 - gp2;
v  = v/norm(v);

q1 = quatBTVec([1 0 0]', v);

if abs(v(3)) < 1e-5
    q = q1;
    return;
end

u1 = quatOnVec([0 1 0]', q1);
u  = cross(v, [0 0 1]');

if norm(u) < 1e-5
    % grasp axis is vertical....
    % q should get rejected in planning anyway
    q = q1;
    return;
end

% get the rotation that rotates u1 to u
% then apply it to q1
qg = quatBTVec(u1, u);
q  = quatMTimes(qg, q1);

% check, there are two u, one of them will turn gravity upwards 
g1 = quatOnVec([0 0 -1]', q);
if g1(3) > 0
	% redo with -u
	u  = -u;
	qg = quatBTVec(u1, u);
	q  = quatMTimes(qg, q1);
end

% % check:
% v
% quatOnVec([1 0 0]', q)
% % quatOnVec([0 1 0]', q)
end