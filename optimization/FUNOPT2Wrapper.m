function [f, g] = FUNOPT2Wrapper(x)
    global paraOpt2;
	[g, f] = FUNOPT2_Jac(x, paraOpt2);
end