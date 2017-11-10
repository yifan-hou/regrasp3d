% find a rotated qr that is closest 
% qrg = min_x angBTQuat(q0, qr(x)*qg)
% where qr(x) = aa2quat(x, ax)
% 
% 	
function qrg = getClosestQuat(q0, qg, ax)
Qg = [qg(1) -qg(2) -qg(3) -qg(4);
	  qg(2)  qg(1)  qg(4) -qg(3);
	  qg(3) -qg(4)  qg(1)  qg(2);
	  qg(4) qg(3)  -qg(2)  qg(1)];
m = q0'*Qg;


% angBTQuat(q0, qr(x)*qg) = acos(q0'*qr(x)*qg)
% MAX q0'*qr(x)*qg = (q0'*Qg)*qr(x) = m*qr(x) 
% = m1cos(x/2) + (m2*ax1 + m3*ax2 + m4*ax3)sin(x/2) = acos(x/2) + bsin(x/2) = sqrt(a^2 + b^2)sin(x+phi)
a = m(1);
b = m(2)*ax(1) + m(3)*ax(2) + m(4)*ax(3);

phi = atan2(a, b);
x = pi/2 - phi;

qr = aa2quat(x, ax);
qrg = quatMTimes(qr, qg);


end