function con = checkConstraint(x, paraOpt, FUN, Flow, Fupp)

f = FUN(x, paraOpt);
cost = f(1);
con = zeros(length(f), 2);
for i = 1:length(f)
	if f(i) < Flow(i)
		con(i, 1) = Flow(i) - f(i);
	end

	if f(i) > Fupp(i)
		con(i, 2) = f(i) - Fupp(i);
	end
end