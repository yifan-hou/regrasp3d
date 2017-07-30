function plotObject(mesh, fid, q)
if nargin <= 2
	q = [1 0 0 0]';
end

if nargin <= 1
	fid = 1;
end

for i = 1:size(mesh.points,2)
	mesh.points(:,i) = quatOnVec(mesh.points(:,i), q);
end

figure(fid);clf;hold on;
plot3(mesh.points(1,:), mesh.points(2,:), mesh.points(3,:), '.');
text(mesh.points(1,1),mesh.points(2,1),mesh.points(3,1),  'p1');
text(mesh.points(1,2),mesh.points(2,2),mesh.points(3,2),  'p2');
text(mesh.points(1,3),mesh.points(2,3),mesh.points(3,3),  'p3');
text(mesh.points(1,4),mesh.points(2,4),mesh.points(3,4),  'p4');
text(mesh.points(1,5),mesh.points(2,5),mesh.points(3,5),  'p5');
text(mesh.points(1,6),mesh.points(2,6),mesh.points(3,6),  'p6');
text(mesh.points(1,7),mesh.points(2,7),mesh.points(3,7),  'p7');
text(mesh.points(1,8),mesh.points(2,8),mesh.points(3,8),  'p8');
trisurf(mesh.faces',mesh.points(1,:), mesh.points(2,:), mesh.points(3,:), 'FaceAlpha', 0.3);
xlabel('X'); ylabel('Y'); zlabel('Z');

axis equal;

view(-43, 27);

end