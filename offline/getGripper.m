% get gripper model for collisition checking
% gipper is consisted of two fingertips, two fingers and a palm
% Gripper axis is in +x direction
% palm is to the top of the fingers.
% vertices_list: fingertip_plus, fingertip_minus, palm
function [gripper] = getGripper() 
FINGERTIP_RADIUS  = 10;
FIGNERTIP_EDGES     = 15;
FIGNERTIP_THICKNESS = 9.3;
FINGER_WIDTH        = 20;
FINGER_THICKNESS    = 6.7;
PALM_HEIGHT         = 60;
PALM_WIDTH          = 49;

vertices_list      = cell(5,1);
vertices_safe_list = cell(5,1);
faces_list         = cell(5,1);

% fingertips
pad      = zeros(3, FIGNERTIP_EDGES); % in y-z plane
angles   = 2*pi*(1:FIGNERTIP_EDGES)/FIGNERTIP_EDGES;
pad(2,:) = FINGERTIP_RADIUS*sin(angles);
pad(3,:) = FINGERTIP_RADIUS*cos(angles);

pad_minus = pad; pad_minus(1, :) = pad_minus(1, :) - FIGNERTIP_THICKNESS;
pad_plus  = pad; pad_plus(1, :)  = pad_plus(1, :)  + FIGNERTIP_THICKNESS;

pad_minus_safe = pad; pad_minus_safe(1, :) = pad_minus_safe(1, :) - 10*FIGNERTIP_THICKNESS;
pad_plus_safe  = pad; pad_plus_safe(1, :)  = pad_plus_safe(1, :)  + 10*FIGNERTIP_THICKNESS;

% finger
side       = zeros(3, 4);
side(2, :) = [-FINGER_WIDTH/2, FINGER_WIDTH/2, FINGER_WIDTH/2, -FINGER_WIDTH/2];
side(3, :) = [0 0 PALM_HEIGHT PALM_HEIGHT];

side_plus  = side; side_plus(1, :)  = side_plus(1, :)  + FINGER_THICKNESS + FIGNERTIP_THICKNESS;
side_minus = side; side_minus(1, :) = side_minus(1, :) - FINGER_THICKNESS - FIGNERTIP_THICKNESS;

side_plus_safe  = side; side_plus_safe(1, :)  = side_plus_safe(1, :)  + 10*FINGER_THICKNESS + FIGNERTIP_THICKNESS;
side_minus_safe = side; side_minus_safe(1, :) = side_minus_safe(1, :) - 10*FINGER_THICKNESS - FIGNERTIP_THICKNESS;

side_plus_inner   = side; side_plus_inner(1, :)   = side_plus_inner(1, :)  + FIGNERTIP_THICKNESS;
side_minus_inner  = side; side_minus_inner(1, :)  = side_minus_inner(1, :) - FIGNERTIP_THICKNESS;

% palm
palm_side        = zeros(3, 4);
palm_side(2, :)  = [-PALM_WIDTH/2, PALM_WIDTH/2, PALM_WIDTH/2, -PALM_WIDTH/2];
palm_side(3, :)  = PALM_HEIGHT + [0 0 0.2 0.2];
palm_plus        = palm_side; 
palm_plus(1, :)  = palm_plus(1, :) + FIGNERTIP_THICKNESS + FINGER_THICKNESS; 
palm_minus       = palm_side; 
palm_minus(1, :) = palm_minus(1, :) - FIGNERTIP_THICKNESS - FINGER_THICKNESS; 

vertices_list{1} = [pad pad_plus ];
vertices_list{2} = [pad pad_minus];
vertices_list{3} = [side_plus_inner side_plus];
vertices_list{4} = [side_minus_inner side_minus];
vertices_list{5} = palm_plus;
vertices_list{6} = palm_minus;

vertices_safe_list{1} = [pad pad_plus_safe ];
vertices_safe_list{2} = [pad pad_minus_safe];
vertices_safe_list{3} = [side_plus_inner side_plus_safe];
vertices_safe_list{4} = [side_minus_inner side_minus_safe];
vertices_safe_list{5} = palm_plus;
vertices_safe_list{6} = palm_minus;

faces_list{1}    = convhull(vertices_list{1}', 'simplify',true);
faces_list{2}    = convhull(vertices_list{2}', 'simplify',true);
faces_list{3}    = convhull(vertices_list{3}', 'simplify',true);
faces_list{4}    = convhull(vertices_list{4}', 'simplify',true);
faces_list{5}    = convhull([palm_plus palm_minus]', 'simplify',true);

gripper.vertices      = vertices_list;
gripper.vertices_safe = vertices_safe_list;
gripper.faces         = faces_list; 


% % debug

% % Note
% % vertices_list is used for visualization
% % vertices_safe_list is used for collision checking

% figure(1);clf;hold on;
% trisurf(gripper.faces{1}, vertices_list{1}(1,:), vertices_list{1}(2,:), vertices_list{1}(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.3);
% trisurf(gripper.faces{2}, vertices_list{2}(1,:), vertices_list{2}(2,:), vertices_list{2}(3,:), 'Facecolor', 'k', 'FaceAlpha', 0.3);
% trisurf(gripper.faces{3}, vertices_list{3}(1,:), vertices_list{3}(2,:), vertices_list{3}(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.7);
% trisurf(gripper.faces{4}, vertices_list{4}(1,:), vertices_list{4}(2,:), vertices_list{4}(3,:), 'Facecolor', 'b', 'FaceAlpha', 0.7);
% trisurf(gripper.faces{5}, [palm_plus(1,:) palm_side(1,:)], [palm_plus(2,:) palm_side(2,:)], [palm_plus(3,:) palm_side(3,:)], 'Facecolor', 'b', 'FaceAlpha', 0.7);
% axis equal;
% xlabel('x');
% ylabel('y');
% zlabel('z');