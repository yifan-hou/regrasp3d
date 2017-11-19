clear;clc

addpath ../optimization/generated/
N = 50;

para_delta_q       = 0.05;
para_q0            = [1 0 0 0]';
para_qf            = [0 1 0 0]';
para_Gp1o          = [0.8 0.5 0.4]';
para_Gp2o          = [0.8 0.6 0.4]';
para_GpZ_limit     = 0.1;
para_Gp_tilt_limit = 50*pi/180;

para_cost_goal_k = 10;
para_cost_tilt_k = 1;


for i = 1:20

	nq = rand(4, 150);
	nq = bsxfun(@rdivide, nq, normByCol(nq));

	tic;
	FUN = FUN1(nq(:), para_delta_q, para_q0, para_qf, para_Gp1o, para_Gp2o, para_GpZ_limit, para_Gp_tilt_limit, para_cost_goal_k, para_cost_tilt_k);
	toc;
end






% clc;

% NFound = [174	2;
% 		 337	41;
% 		 611	173;
% 		 671	363;
% 		 713	500;
% 		 725	610;
% 		 768	695;
% 		 749	741]';
% Extime = [41.4655		50;
% 		98.4777		82.6829;
% 		130.9656		88.8844;
% 		131.1773		95.4077;
% 		128.6536		100.198;
% 		130.9807		102.9836;
% 		131.8997		100.8173;
% 		136.1816		103.5398]';

% total_rotation = [3.2699		4.3241;
% 				6.8412		7.7307;
% 				8.7031		8.1018;
% 				8.8678		8.6221;
% 				9.0394		8.8962;
% 				9.2327		9.0493;
% 				9.3738		8.9722;
% 				9.7076		9.1904]';

% total_translation = [1.1092		0.83562;
% 					2.2269		1.6941;
% 					2.5971		1.6583;
% 					2.488		1.9842;
% 					2.3786		2.084;
% 					2.2419		2.0285;
% 					2.1997		2.0344;
% 					2.2895		2.1084]';

% NRegrasp = [1.1552		1.5;
% 			1.9674		2.7561;
% 			2.198		2.6069;
% 			2.1028		2.708;
% 			2.1262		2.62;
% 			2.0993		2.5639;
% 			2.125		2.4647;
% 			2.1535		2.4494]';


% % NFound: 677	650
% % Total rotation: 8.6302		8.9874
% % Total translation: 1.9471		2.0286
% % # of regrasp: 1.9338		2.4169

% % 2x8
% tilt_angle = [1:8]*10;

% blue   = [49,130,189]/255;
% orange = [230,85,13]/255;

% figure(1);clf(1);hold on;
% plot(tilt_angle, NFound(1,:),'.-', 'Color', blue, 'markersize',15, 'linewidth', 1.5);
% plot(tilt_angle, NFound(2,:),'.-', 'Color', orange, 'markersize',15, 'linewidth', 1.5);
% ylabel('Number')
% legend('Pivoting', 'Pick&Place');
% legend('boxoff');

% figure(2);clf(2);hold on;
% plot(tilt_angle, total_rotation(1,:),'.-', 'Color', blue, 'markersize',15, 'linewidth', 1.5);
% plot(tilt_angle, total_rotation(2,:),'.-', 'Color', orange, 'markersize',15, 'linewidth', 1.5);
% ylabel('Angle (rad)')
% legend('Pivoting', 'Pick&Place');
% legend('boxoff');

% figure(3);clf(3);hold on;
% plot(tilt_angle, total_translation(1,:),'.-', 'Color', blue, 'markersize',15, 'linewidth', 1.5);
% plot(tilt_angle, total_translation(2,:),'.-', 'Color', orange, 'markersize',15, 'linewidth', 1.5);
% ylabel('Distance (m)')
% legend('Pivoting', 'Pick&Place');
% legend('boxoff');

% figure(4);clf(4);hold on;
% plot(tilt_angle, NRegrasp(1,:),'.-', 'Color', blue, 'markersize',15, 'linewidth', 1.5);
% plot(tilt_angle, NRegrasp(2,:),'.-', 'Color', orange, 'markersize',15, 'linewidth', 1.5);
% ylabel('Number')
% legend('Pivoting', 'Pick&Place');
% legend('boxoff');