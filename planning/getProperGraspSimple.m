% q moves [1 0 0] to (gp1 - gp2), [0 1 0] to [? ? 0]
% Inputs:
%   gp1, gp2: 3x1, grasp pos in world frame
% Outputs:
%   qg: the grasp frame
%   v: the grasp axis

function [qg, v] = getProperGraspSimple(gp1, gp2)
v  = gp1 - gp2;
v  = v/norm(v);

q1 = quatBTVec([1 0 0]', v);

if abs(v(3)) < 1e-5
    qg = q1;
else
    u1 = quatOnVec([0 1 0]', q1);
    % u  = cross(v, [0 0 1]'); % wrong
    u  = cross([0 0 1]', v);

    if norm(u) < 1e-5
        % grasp axis is vertical....
        % qg should get rejected in planning anyway
        qg = q1;
    else
        % get the rotation that rotates u1 to u
        % then apply it to q1
        qtemp = quatBTVec(u1, u);
        qg    = quatMTimes(qtemp, q1);

        % check, there are two u, one of them will turn gravity upwards 
        g1 = quatOnVec([0 0 -1]', qg);
        if g1(3) > 0
        	% redo with -u
            u     = -u;
            qtemp = quatBTVec(u1, u);
            qg    = quatMTimes(qtemp, q1);
        end
    end
    
end


% % check:
% v
% quatOnVec([1 0 0]', qg)