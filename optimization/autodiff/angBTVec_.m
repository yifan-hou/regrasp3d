function ang = angBTVec_(x,b)
x   = x/norm_(x);
b   = b/norm_(b);
ang = acos(dot(x, b));

end