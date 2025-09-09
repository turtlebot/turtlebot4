^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot4_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.1 (2025-09-09)
------------------

2.1.0 (2025-05-23)
------------------
* Fix tag order
* Update package maintainers
* Contributors: Chris Iverach-Brereton

2.0.1 (2024-09-25)
------------------
* Block the wall-follow callbacks from running if the robot is presently docked
* Contributors: Chris Iverach-Brereton

2.0.0 (2024-08-29)
------------------

1.0.5 (2024-07-02)
------------------

1.0.4 (2023-11-08)
------------------

1.0.3 (2023-05-31)
------------------

1.0.2 (2023-05-15)
------------------

1.0.1 (2023-02-28)
------------------
* Force 1HZ update on display
* Contributors: Roni Kreinin

1.0.0 (2023-02-17)
------------------
* Updates for new DepthAI node.
* Linter fixes
* Separated RPLIDAR Motor function into stop/start
* Added OAKD stop/start function
* Added power saver option
* Update display only when something has changed
* Function calls
* Namespacing
* Flash Comms LED based on high frequency wheel status feedback instead of low frequency battery state
* Move comms_timer() from battery_callback() to wheel_status_callback()
* Updated dock action
* Contributors: Joey Yang, Roni Kreinin

0.1.2 (2022-09-15)
------------------
* Added support for Empty service
* Added RPLIDAR motor stop service as a function option
* Added timeouts to services (defaults to 30s)
* Updated rclcpp action api
* Contributors: Daisuke Nishimatsu, Roni Kreinin

0.1.1 (2022-07-12)
------------------
* Merge pull request `#7 <https://github.com/turtlebot/turtlebot4/issues/7>`_ from turtlebot/roni-kreinin/linters
  Fixed linter errors
* Fixed linter errors
* Contributors: Roni Kreinin, roni-kreinin

0.1.0 (2022-05-03)
------------------
* First Galactic release
* Contributors: Roni Kreinin, ahcorde
