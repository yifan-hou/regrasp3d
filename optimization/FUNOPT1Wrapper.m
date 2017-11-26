function [f, g] = FUNOPT1Wrapper(x)
    global paraOpt;
	[g, f] = FUNOPT1_Jac(x, paraOpt);
end