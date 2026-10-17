# Remy coordinate and motion check

The existing asset import rotations are +90 degrees about X, then +90 about Z.
They map source `(x,y,z)` to robot `(z,x,y)`. The roller Tube node in `model_2.glb`
is at source `(-0.0099390533, 0.2032626419, 0.3152833843)` metres: the roller is on
robot **+X**. No asset or chassis heading rotation is needed to make it deploy outward.

Rack position is linear extension in metres. Component 2 translates by
`(+extension*cos(7.5 degrees), 0, -extension*sin(7.5 degrees))`; kicker and hopper
components 3 and 4 translate by the same X displacement. Stow/safe/deploy values
are 0, 0.13, and 0.30 m. Test those poses and check floor clearance and alignment
against the physical rack. The flywheel pivot remains approximately (-0.103310,0,0.423) m.

Simulation pickup and projectile yaw follow +X. The shooter release position
remains the existing estimate (-0.19,0,0.45) m; exit position and firing direction
are separate quantities. Real aiming/camera yaw remains unchanged at 180 degrees
pending a physical robot-relative forward check. Do not infer real inversion or
encoder zero from a visual-model change.

Select Remy with `Visualization/RobotRemy/RobotPose` and its six-element
`ComponentPoses` child. Restart the updated simulator before judging motion;
old logs retain the old transforms.
