^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ackermann_vehicle_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2026-08-03)
------------------
* Made controller loading deterministic on Noetic and added headless launch
  arguments for reproducible Gazebo checks.
* Corrected the joy launcher's command-converter package.
* Declared the navigation package dependency required by the joy launcher.

0.1.3 (2015-09-18)
------------------
* Added some run_depend elements, and deleted some others.
* Added a copyright comment.

0.1.2 (2015-08-19)
------------------
* In the parameter and argument documentation, changed "float" to "double".
* Changed the name of ackermann_controller.py to ackermann_controller. This was
  done because the catkin documentation states that executable script names
  generally do not include a .py suffix.
* Scripts, configuration files, and launch files are now installed.
* Changed the name of "nodes" to "scripts".

0.1.1 (2014-09-26)
------------------

0.1.0 (2014-09-18)
------------------
* Initial release.
