function ang = tiltedConeAng(cone_half_ang, tilted_ang)
H = 1;
R = tan(cone_half_ang)*H;
r = tan(tilted_ang)*H;
L = sqrt(R^2 + H^2);
m = sqrt(r^2 + H^2);
theta = asin(r/R);
h = R*cos(theta);

ang = sss2aaa(h, L, m);

end