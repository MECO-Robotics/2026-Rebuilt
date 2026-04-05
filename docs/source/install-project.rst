Install Project
===============

Prepare your environment and run the project.


Install tools
-------------

Install the standard FRC development tools before opening the project.

- **WPILib 2026**
  Install the `WPILib VS Code bundle <https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html>`_ for the 2026 season. This provides Gradle, the Java toolchain, simulation tools, and the WPILib extension used to build and deploy the robot code.
- **FRC Game Tools**
  Install the current `FRC Game Tools <https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html>`_ package if you plan to deploy to a RoboRIO or connect to robot hardware from your laptop.

Clone and build
---------------

After the tools are installed, bring the project onto your machine and verify that it builds before making any changes.

1. Clone the repository.
2. Open the project folder in VS Code.
3. Allow WPILib to finish indexing the project and downloading dependencies if prompted.
4. Build the project using the WPILib build command or by running ``./gradlew build`` from the project root.

If the project does not build cleanly at this stage, fix the environment problem before moving on to robot configuration or code changes.


Deploy
------

Once the project builds, choose the fastest way to confirm it runs:

- **Run in simulation**
  Use this first if you are only verifying that the code starts correctly on your machine.
- **Deploy to robot**
  Deploy only after you have updated hardware config and mechanism gains and limits to match your robot.


Expected result
---------------

- Project builds without errors
- The robot code starts in simulation or deploys successfully to the RoboRIO
- No immediate startup crashes appear in the console or driver station


Common setup issues
-------------------

- **Wrong WPILib version**
  If the installed WPILib version does not match the project season, builds and vendor dependencies may fail.
- **Missing vendor dependencies**
  If CTRE, REV, or other vendor libraries are not installed correctly, the project may fail during dependency resolution or compilation.
- **VS Code opened in the wrong folder**
  Open the repository root, not just the ``docs/`` folder or a nested source directory.
- **Java or Gradle still initializing**
  On first open, give WPILib time to finish setup before assuming the project is broken.
- **Robot-specific constants not configured**
  A successful build does not mean the project is ready to run on your robot. Update IDs, inversion, limits, and mechanism gains before hardware testing.
