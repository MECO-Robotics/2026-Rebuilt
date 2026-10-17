# Detailed CAD intake animation

This separate AdvantageScope asset preserves the existing working `Robot_16thNote` model.
It combines the October 5 drive-base and detailed shooter STEP exports and separates
the intake into `model_0.glb`. Select **16th Note Detailed** to inspect it.
Without a Component pose source, the intake stays in its exported CAD position.
To animate it, add `Visualization/16thNote/ComponentPoses` under the robot as
**Component**, using the **Pose3d[]** source from AdvantageKit RealOutputs.
Restart simulation to publish the new topic. The user confirmed that the CAD
shows intake-down and confirmed **160 degrees** between stowed and deployed.
The simulator uses zero at stowed and 160 degrees at deployed.


## Geometry and assumptions

- STEP millimeters converted to meters; CAD +Y maps to robot +X (intake side),
  CAD +X maps to robot -Y, and CAD +Z stays up.
- Drive base and mechanisms are centered using their exported origins. The whole
  model is lifted 0.0126517017 m to put the lowest chassis geometry on the floor.
  Mounting alignment and actual tire contact height still need visual confirmation.
- Candidate intake pivot in robot coordinates: (0.0965173098, 0, 0.3551707017) m,
  through the shared jackshaft/indexer axis, parallel to robot Y.
- Moving mesh: left/right intake plates, outer intake roller, roller backer,
  second roller belt and jackshaft. The first roller belt and pivot chain span
  fixed motor locations and stay with the stationary mesh. No belt circulation
  or roller rotation is animated.
- Omitted CAD helpers: origin cube, example fuel, trajectory curve, centered
  duplicate belt/chain design outlines, Pivoty Plate and Part 38 (unconfirmed
  center-plane design geometry). These omissions are for visualization only.
- Colors are visualization colors assigned by part name, not CAD materials.
- This is a visual asset, not a source of measured hardware travel, motor gains,
  mass/inertia, pickup collision geometry or projectile calibration.

## Remaining physical validation

Verify chassis/mechanism mounting alignment and physical stop clearance. The
published poses use the candidate pivot above and rotation about robot Y of
`simulatedIntakeDegrees - 160`. At 160 degrees the intake matches the CAD;
at zero it rotates upward 160 degrees to stowed. The normal SIM feed target
is 5 degrees, slightly clear of the upper stop; that clearance is still an
example requiring physical verification.
No hardware constants are changed by this asset.

Reference: https://docs.advantagescope.org/more-features/custom-assets/

## Export compatibility

Both GLB files must contain explicit `POSITION` and `NORMAL` attributes on every
mesh primitive. AdvantageScope 26 skips geometry without normals even when the
pose topic is valid. Export with `include_normals=True` when using trimesh, and
use explicit material base colors (the loader does not preserve vertex colors
in its geometry-merging path). After replacing assets, use View > Reload, then
File > Connect to Simulator to resume live data.
