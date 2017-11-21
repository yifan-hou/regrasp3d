function vr = quatOnVec_(v,q)
	A = [0 0 0;
		 1 0 0;
		 0 1 0;
		 0 0 1];
	v_    = A*v;
	temp1 = quatMTimes_1(q, v_);
	temp2 = quatInv(q);
	vr_   = quatMTimes_1(temp1, temp2);
	vr    = vr_(2:4);
end