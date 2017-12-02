function [f, g] = FUNOPT_OBJWrapper(x)
    global paraOpt_OBJ;
	[g, f] = FUNOPT_OBJ_Jac(x, paraOpt_OBJ);
end