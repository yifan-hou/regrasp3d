function [f, g] = FUNOPT2Wrapper(x)
    global paraOpt;
	[g, f] = FUNOPT2_Jac(x, paraOpt);
end