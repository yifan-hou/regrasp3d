# regrasp3d

# Planning
###XY motion:
XY motion of gripper is computed during planObject.
It is recorded incrementally in grpxy_delta.
During sticking in pivoting, the object rotates about contact point.
Otherwise, grpxy_delta is set to zero.

# Handle translational offset
In all planning, translational offset is ignored (i.e. the object start from x=0, y=0, on a table with z=0)
In ploting and generateTraj, offset is considered.


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

