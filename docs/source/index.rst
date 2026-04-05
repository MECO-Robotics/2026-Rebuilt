2026 Base Bot
=============

Official documentation for MECO Robotics **Base Project**.

Included are both beginner-oriented guides as well as more in-depth documentation for maintenance and modification purposes.

.. toctree::
   :maxdepth: 2
   :titlesonly:
   :caption: Start Here
   :includehidden:

   install-project
   mechanism-types
   configure-robot
   judging
   command-compositions
   tune-iterate
   troubleshooting
   
.. toctree::
   :maxdepth: 1
   :titlesonly:
   :caption: Reference

   architecture
   subsystems
   customizing


Quick start
-----------

- Installing project locally: :doc:`install-project`
- Choosing the correct mechanism pattern: :doc:`mechanism-types`
- Adapting robot to your hardware: :doc:`configure-robot`
- Basic information for judges: :doc:`judging`
- Write commands to control the robot: :doc:`command-compositions`
- Troubleshooting: :doc:`troubleshooting`
- Ready to improve behavior: :doc:`tune-iterate`


Rules
-----

- Get the robot working before optimizing
- Do not rewrite subsystem internals early
- Do not modify IO layers unless hardware requires it
