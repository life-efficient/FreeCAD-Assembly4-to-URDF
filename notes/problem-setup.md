Modeling is done in FreeCAD

FreeCAD joints specify each joint with two placements:
1. from the origin of the child link to the position and orientation on its mesh where the joint is located
2. from the origin of the parent link to the position and orientation on its mesh where the joint is located
which of these is the parent and child is determined programmatically - this is not an area for focus, as the joint tree prints perfectly
However, althouth both vectors point to the same place, they may have a different rotation (e.g. placed back to back rather than face to face). Because of this, i've tried to align their axes by taking the rotation difference from one to anohter and applying that at the end of each urdf joint i create

URDF joints are a little different:
they point from joint to joint, not from link origin to link origin
my links are not attached to joints at their origin, so a mesh offset is applied
e.g. one of the modelled parts is a plate with its origin on a corner, and uniformly spread mounting holes where parts are attached to along its length - so mesh offset is necessary

Currently, the joints correctly connect the right parts of each link, but the parts are misaligned. a rotation about the joint axis would fix the way that the parts are mated. But I'm finding it really hard to determing what rotation is missing or needs to be applied

clues of issue: 
- the correct part of each link is coincident with the joint it connects to on the other link
- so parts show connected at the right spots, but twisted incorrectly about the joint axis
- rotation about their origin (changing the link rpy values will not fix this, as that rotates them about their mesh origin, not the joint)
this suggests that the resulting end frame of each joint is misoriented

its not just that parts are misaligned