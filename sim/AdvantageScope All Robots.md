# One layout for REBUILT and 16th Note

Import **AdvantageScope All Robots.json** from the repository root using AdvantageScope's **File → Import Layout**. This replaces the open layout; export any personal layout you want to keep first. The combined layout includes the existing REBUILT system-check controls and 3D view, plus driving and fuel views for both robots.

Tabs starting with **REBUILT |** cover the Remy 3D model, swerve measured/target states, drive telemetry, automated pit-check controls, health, reports, simulation fault injection, shooting/intake, and rack diagnostics.

Tabs starting with **16th Note |** cover the detailed 3D model, tank drive, fuel status, commissioning, mechanism configuration, fuel tuning, autonomous settings, and simulation practice commands. 16th Note uses its existing Test-mode commissioning controls; this layout does not add REBUILT's automated test sequence to that robot.

Keep this layout open and change the NT4 connection when switching robots. For simulation, connect to `127.0.0.1` and run one simulator at a time. For hardware, use the address of the robot you intend to operate. A tab name does not select or isolate a robot connection. Fields for the other robot will be missing or crossed out, which is expected. Real 16th Note wheel-position/heading feedback is not configured; its simulated pose and fuel world are simulation-only.

Both custom assets must already be installed in AdvantageScope: **Remy** and **16th Note Detailed**. The layout references their names; it does not bundle the model files. Both 3D views include their component poses so intake movement remains visible. The individual swerve wheels and some rollers are not separately animated CAD components.

Start REBUILT checks in **REBUILT | Pit Check Controls**, then switch to **REBUILT | Remy System Check 3D**. Existing Test-mode, arming, and safety requirements still apply. The browser report/live-monitor URLs are in **REBUILT | Reports & CAN**.

16th Note chooser entries expose `options`, `selected`, and `active`: publish one of the listed options to `selected`, then verify `active`. Disabled-only command entries expose `running`; publish true to request them and inspect ActionStatus/ResetStatus. Commissioning still requires Test mode and the RB/R1 deadman. Importing a layout does not publish control values or start tests.

All fields use live `NT:/` paths. This layout targets live driving and system checks; offline WPILOG paths may need remapping. No robot code or control bindings are changed by importing it.
