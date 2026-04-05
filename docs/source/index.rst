2026 Base Bot
=============

Official documentation for MECO Robotics **Base Project**.

Included are beginner-oriented guides and more in-depth documentation for maintenance and modification purposes.

.. toctree::
   :maxdepth: 2
   :titlesonly:
   :caption: Start Here
   :includehidden:

   setup-build
   first-30-minutes
   configure-robot
   bringup-test
   tune-iterate

.. toctree::
   :maxdepth: 1
   :titlesonly:
   :caption: Reference

   architecture
   subsystems
   customizing


Quick start
-----------

- Installing project locally: :doc:`setup-build`
- Adapting robot to your hardware: :doc:`configure-robot`
- Using the codebase: :doc:`first-30-minutes`
- Robot not working: :doc:`bringup-test`
- Ready to improve behavior: :doc:`tune-iterate`


Rules
-----

- Get the robot working before optimizing
- Do not rewrite subsystem internals early
- Do not modify IO layers unless hardware requires it