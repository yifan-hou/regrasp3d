# regrasp3d

# Planning
###XY motion:
XY motion of gripper is computed during planObject.
It is recorded incrementally in grpxy_delta.
During sticking in pivoting, the object rotates about contact point.
Otherwise, grpxy_delta is set to zero.

# Online execution
## Orientation
move according to plan. (pos)

## Translation:
Firm grasp:
	xy: zero force
	z: 0.9mg lift up force 
Pivoting:
	Sticking: it really has to be sticking, otherwise the plan breaks down
		xy: position, follow the plan, rotate about axis
		z: 0 force, or press down
	(about to) Slipping: 
		xy: force, towards the origin. (not implemented yet)
		z: position, move according to plan
