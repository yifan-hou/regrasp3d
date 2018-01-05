function clickGrasp(mesh, gripper, para)
disp('[ClickGrasp] User picking Grasp Positions.');
area_bin             = mesh.area/sum(mesh.area);

Np            = 3000;
samples       = mnrnd(Np, area_bin);
sample_points = zeros(3, Np);
id_sampled    = find(samples > 0);
p1            = zeros(3); % the sampled triangle

count = 1;
id_points = zeros(1, Np);
for f = 1:length(id_sampled)
	N = samples(id_sampled(f));
	p1(:,1)     = mesh.vertices(mesh.faces(id_sampled(f), 1), :)';
	p1(:,2)     = mesh.vertices(mesh.faces(id_sampled(f), 2), :)';
	p1(:,3)     = mesh.vertices(mesh.faces(id_sampled(f), 3), :)';

	sample_points(:, count:count+N-1) = sampleTriUniform(p1(:,1), p1(:,2),p1(:,3), N);
	id_points(count:count+N-1) = id_sampled(f);
	count = count + N;
end

grasps.count     = 0;
grasps.points    = zeros(3, para.NGS, 2);
grasps.range     = zeros(360, para.NGS);
grasps.ref_frame = zeros(4, para.NGS); 

delete clickData.mat
save clickData.mat sample_points id_points grasps

% show the point cloud
h = gcf;
figure(1);
plotObject(mesh, 1); hold on;
plot3(sample_points(1,:), sample_points(2,:), sample_points(3,:), 'k.'); 
cameratoolbar('Show'); % show the camera toolbar
hold on; % so we can highlight clicked points without clearing the figure
% set the callback, pass sample_points to the callback function
set(h, 'WindowButtonDownFcn', {@checkClickPoint, sample_points, mesh, gripper, para}); 




