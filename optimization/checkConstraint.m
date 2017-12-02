function [con_x, con_f, cost] = checkConstraint(x, paraOpt, FUN, xlow, xupp, Flow, Fupp)

f = FUN(x, paraOpt);

cost = f(1);
con_x = zeros(length(x), 2);
con_f = zeros(length(f), 2);
for i = 1:length(x)
	if x(i) < xlow(i)
		con_x(i, 1) = xlow(i) - x(i);
	end

	if x(i) > xupp(i)
		con_x(i, 2) = x(i) - xupp(i);
	end
end

for i = 1:length(f)
	if f(i) < Flow(i)
		con_f(i, 1) = Flow(i) - f(i);
	end

	if f(i) > Fupp(i)
		con_f(i, 2) = f(i) - Fupp(i);
	end
end