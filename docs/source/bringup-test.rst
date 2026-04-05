Troubleshooting
===============

Common issues and how to fix them.


If mechanism isn't running
------------------------

Check wiring
~~~~~~~~~~~~~~~~~
Often the issue is a simple wiring problem. Check:

- RoboRIO has power
- Motor controllers have power
- CAN wiring is correct
- Motors are connected to the correct CAN bus

Check Hardware Configuration
~~~~~~~~~~~~~~~~~~~~~~~
Another common issue is incorrect hardware configuration. Check:

- CAN IDs match constants
- Motor type is correct in constants
- Motor controllers are flashed with correct firmware
- CAN conflicts
