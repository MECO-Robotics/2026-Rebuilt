Game-Piece Camera Autostart
===========================

The robot reads game-piece detections from HTTP camera servers. Each camera computer should start its
``camera.py`` server when it boots so the robot can connect as soon as it is enabled.

Expected camera addresses
-------------------------

The current robot constants expect these camera servers:

- Front camera: ``http://10.83.24.11:5800``
- Rear camera: ``http://10.83.24.12:5800``

If the camera computers use different static IP addresses, update
``src/main/java/frc/robot/constants/vision/PieceDetectionConstants.java`` before deploying robot code.

Manual test
-----------

On the camera computer, run:

.. code-block:: bash

   cd ~/Game-Piece-Detection
   python3 camera.py

From another computer on the robot network, open:

.. code-block:: text

   http://10.83.24.11:5800
   http://10.83.24.11:5800/data
   http://10.83.24.11:5800/video

Use the rear camera IP instead when testing the rear camera.

Linux boot service
------------------

Use this on the Beelink or other Linux coprocessor that runs the camera script.

1. Put the camera repo at ``/home/<your-user>/Game-Piece-Detection``.
2. Create a Python virtual environment and install the camera dependencies.

.. code-block:: bash

   cd /home/<your-user>/Game-Piece-Detection
   python3 -m venv .venv
   .venv/bin/python -m pip install flask opencv-python numpy

3. Copy the service template from this robot repo and edit paths/user if needed.

.. code-block:: bash

   sudo cp /path/to/2026-Rebuilt/tools/camera/game-piece-camera.service /etc/systemd/system/game-piece-camera.service
   SERVICE_FILE=/etc/systemd/system/game-piece-camera.service
   USER_FOR_CAMERA="${SUDO_USER:-$USER}"
   GROUP_FOR_CAMERA="$(id -gn "$USER_FOR_CAMERA")"
   sudo sed -i "s|replace-with-your-user|$USER_FOR_CAMERA|g; s|replace-with-your-group|$GROUP_FOR_CAMERA|g" "$SERVICE_FILE"

The template contains placeholder strings in three lines:

- Linux user/group: ``replace-with-your-user`` and your matching group
- Camera repo: ``/home/<your-user>/Game-Piece-Detection``
- Python executable: ``/home/<your-user>/Game-Piece-Detection/.venv/bin/python``

This command reads your current user and group and replaces the placeholders in one step.

4. Enable and start the service.

.. code-block:: bash

   sudo systemctl daemon-reload
   sudo systemctl enable game-piece-camera.service
   sudo systemctl start game-piece-camera.service

5. Check that it is running.

.. code-block:: bash

   systemctl status game-piece-camera.service
   journalctl -u game-piece-camera.service -f

Match workflow
--------------

Before a match:

1. Power on the camera computer.
2. Confirm the camera page opens at ``http://10.83.24.11:5800`` or ``http://10.83.24.12:5800``.
3. Deploy or enable the robot normally.

The robot should not need to start the camera process when enabled. Keeping the camera server alive from boot avoids
camera warmup delays and makes failures easier to see before the match starts.
