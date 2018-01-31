function checkClickPoint(src, eventData, pointCloud, mesh, gripper, para)
disp('Checking new point...')
point = get(gca, 'CurrentPoint'); % mouse click position
camPos = get(gca, 'CameraPosition'); % camera position
camTgt = get(gca, 'CameraTarget'); % where the camera is pointing to

camDir = camPos - camTgt; % camera direction
camUpVect = get(gca, 'CameraUpVector'); % camera 'up' vector

% build an orthonormal frame based on the viewing direction and the 
% up vector (the "view frame")
zAxis = camDir/norm(camDir);    
upAxis = camUpVect/norm(camUpVect); 
xAxis = cross(upAxis, zAxis);
yAxis = cross(zAxis, xAxis);

rot = [xAxis; yAxis; zAxis]; % view rotation 

% the point cloud represented in the view frame
rotatedPointCloud = rot * pointCloud; 

% the clicked point represented in the view frame
rotatedPointFront = rot * point' ;

% there could be multi-layer points. Pick the one to the front
[~, id] = max(rotatedPointFront(3,:));
rotatedPointFront = rotatedPointFront(:, id);

% find the nearest neighbour to the clicked point 
pointCloudIndex = dsearchn(rotatedPointCloud(1:2,:)', ... 
    rotatedPointFront(1:2)');

h = findobj(gca,'Tag','pt'); % try to find the old point
selectedPoint = pointCloud(:, pointCloudIndex); 

if isempty(h) % if it's the first click (i.e. no previous point to delete)
    
    % highlight the selected point
    h = plot3(selectedPoint(1,:), selectedPoint(2,:), ...
        selectedPoint(3,:), 'r.', 'MarkerSize', 20); 
    set(h,'Tag','pt'); % set its Tag property for later use   

else % if it is not the first click

    delete(h); % delete the previously selected point
    
    % highlight the newly selected point
    h = plot3(selectedPoint(1,:), selectedPoint(2,:), ...
        selectedPoint(3,:), 'r.', 'MarkerSize', 20);  
    set(h,'Tag','pt');  % set its Tag property for later use

end


% check this point
load clickData.mat
% sample_points id_points mesh
p1 = selectedPoint;
face_id        = id_points(pointCloudIndex);
triAngle1      = zeros(3); % the sampled triangle
triAngle1(:,1) = mesh.vertices(mesh.faces(face_id, 1), :)';
triAngle1(:,2) = mesh.vertices(mesh.faces(face_id, 2), :)';
triAngle1(:,3) = mesh.vertices(mesh.faces(face_id, 3), :)';
n1             = cross(triAngle1(:,1) - triAngle1(:,2), triAngle1(:,3) - triAngle1(:,2));
n1             = n1/norm(n1);

triAngle2     = zeros(3);
p2 = [];
for j = 1:size(mesh.faces, 1)
    if face_id == j
        continue;
    end
    triAngle2(:,1) = mesh.vertices(mesh.faces(j, 1), :)';
    triAngle2(:,2) = mesh.vertices(mesh.faces(j, 2), :)';
    triAngle2(:,3) = mesh.vertices(mesh.faces(j, 3), :)';
    n2             = cross(triAngle2(:,1) - triAngle2(:,2), triAngle2(:,3) - triAngle2(:,2));
    n2             = n2/norm(n2);
    % check normal
    if abs(n1'*n2) < cos(para.ANGLE_TOL)
        continue;
    end

    % normal is good, sample the other grasp points, check angles
    [pprj, in] = projectOntoTri(triAngle2(:,1), triAngle2(:,2), triAngle2(:,3), selectedPoint);
    if in
        % check angles
        d12 = pprj - selectedPoint;
        if norm(d12) < 1e-3
            continue;
        end
        if (abs(n1'*d12) < cos(1e-3)) || (abs(n2'*d12) < cos(para.ANGLE_TOL))
            continue;
        end
        p2 = [p2 pprj];
    end
end

% 
np2 = size(p2, 2);
p2_id = [];
disp(['number of candidates: ' num2str(np2)]);

a = norm(mesh.COM - p1);
for i = 1:np2
    disp(['- Checking # ' num2str(i)]);
    b        = norm(mesh.COM - p2(:, i));
    c        = norm(p1 - p2(:, i));
    p        = (a+b+c)/2;
    area     = sqrt(p*(p-a)*(p-b)*(p-c));
    dist2COM = area*2/c;
    if dist2COM < para.COM_DIST_LIMIT
        disp('      * Too close to COM.');
        continue;
    end

    % distance is good, do collision detection
    [grasp_feasible_range, grasp_frame] = gripperCollisionCheck(mesh, gripper, [p1 p2(:, i)], para);
    if isempty(grasp_feasible_range)
        disp('      * Collision detection failed.')
        continue;
    end


    disp('      * Works!');
    p2_id = i;
    break;
end

% saving
if ~isempty(p2_id)
    grasps.count                      = grasps.count + 1;
    grasps.points(:, grasps.count, 1) = p1;
    grasps.points(:, grasps.count, 2) = p2(:,p2_id);
    grasps.range(:, grasps.count)     = grasp_feasible_range;
    grasps.ref_frame(:, grasps.count) = grasp_frame;

    save clickData.mat sample_points id_points mesh grasps

    gp = reshape(grasps.points(:,grasps.count,:), [3,2]);
    plot3(gp(1,:), gp(2,:), gp(3,:), '. -','linewidth',2, 'markersize', 20);
    drawnow;
end