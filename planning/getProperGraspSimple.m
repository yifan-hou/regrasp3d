% q moves [1 0 0] to (gp1 - gp2), [0 1 0] to [? ? 0]
% Inputs:
%   gp1, gp2: 3x1, grasp pos in world frame
% Outputs:
%   qg: the grasp frame
%   v: the grasp axis

function [qg, v] = getProperGraspSimple(gp1, gp2)
v  = gp1 - gp2;
v  = v/norm(v);

u  = cross([0 0 1]', v);
unorm = norm(u);
if unorm < 1e-5
    % grasp axis is vertical....
    % qg should get rejected in planning
    if v(3) > 0
        qg = aa2quat(-pi/2, [0 1 0]');
    else
        qg = aa2quat(pi/2, [0 1 0]');
    end
else
    u = u/unorm;
    w = cross(v, u);
    R = [v u w];
    qg = mat2quat(R);
    
%     % check:
%     qgz = quatOnVec([0 0 1]', qg);
%     assert(qgz(3) > 0);
end



