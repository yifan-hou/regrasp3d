% function [Jac,Fun] = FUNOPT_OBJ_Jac(x,para)
% 
% Jacobian wrapper file generated by ADiGator
% ©2010-2014 Matthew J. Weinstein and Anil V. Rao
% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ 
% Contact: mweinstein@ufl.edu
% Bugs/suggestions may be reported to the sourceforge forums
%                    DISCLAIMER
% ADiGator is a general-purpose software distributed under the GNU General
% Public License version 3.0. While the software is distributed with the
% hope that it will be useful, both the software and generated code are
% provided 'AS IS' with NO WARRANTIES OF ANY KIND and no merchantability
% or fitness for any purpose or application.

function [Jac,Fun] = FUNOPT_OBJ_Jac(x,para)
gator_x.f = x;
gator_x.dx = ones(60,1);
FUN = FUNOPT_OBJ_ADiGatorJac(gator_x,para);
Jac = sparse(FUN.dx_location(:,1),FUN.dx_location(:,2),FUN.dx,43,60);
Fun = FUN.f;
end