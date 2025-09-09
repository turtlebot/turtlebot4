^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot4_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.1 (2025-09-09)
------------------
* Fix SLAM parameter passing (`#631 <https://github.com/turtlebot/turtlebot4/issues/631>`_)
  * Instead of manually remapping topics, rewrite the parameters to apply namespaces as needed
  * Add map_name parameter to the config so we can overwrite it correctly
* make footprint topic relative in collision monitor while using with namespace (`#606 <https://github.com/turtlebot/turtlebot4/issues/606>`_)
* Contributors: Chris Iverach-Brereton, Kishan Sawant

2.1.0 (2025-05-23)
------------------
* Change footprint from radius to octagon of similar size (`#592 <https://github.com/turtlebot/turtlebot4/issues/592>`_)
* Nav2 changed how use_sim_time is set, should not be in yaml (`#564 <https://github.com/turtlebot/turtlebot4/issues/564>`_)
* Fix tag order
* Update package maintainers
* Adds support for namespaces to TurtleBot4Navigator.
* Contributors: Chris Iverach-Brereton, Hilary Luo, Pbopbo, Pradyum Aadith

2.0.1 (2024-09-25)
------------------
* Revert to using base_link for navigation; the create3 uses that for odom, and using a different link appears to break slam_toolbox. Update the SLAM launch parameters
* Contributors: Chris Iverach-Brereton

2.0.0 (2024-08-29)
------------------
* Update the SLAM launch file so it works with the latest version of slam_toolbox. Rather than re-impementing all of the launch file, just include the online_sync or online_async launch files as appropriate
* Update nav2 launch configuration to work with the latest jazzy: syntax changes (/ -> ::)
* Use TwistStamped messages for velocity controllers
* Use MPPIController for path follower plugin instead of DWMLocalPlanner
* Assorted minor yaml formatting fixes (e.g. consistent caps for true & false)
* Use base_footprint link as the tf base for slam & localization
* Contributors: Chris Iverach-Brereton

1.0.5 (2024-07-02)
------------------

1.0.4 (2023-11-08)
------------------
* Use opaque function to allow for remapping of scan topics (<https://github.com/turtlebot/turtlebot4/issues/293>)
* Contributors: Hilary Luo

1.0.3 (2023-05-31)
------------------

1.0.2 (2023-05-15)
------------------
* Added namespacing support in sim
* Set warehouse map as default
* New map files for new and modified worlds
* Updating robot radius to max radius and increasing inflation
* Added missing Nav2 plugin
* Contributors: Hilary Luo, Roni Kreinin

1.0.1 (2023-02-28)
------------------
* Navigator fixes
* Contributors: Roni Kreinin

1.0.0 (2023-02-17)
------------------
* Namespacing
* Linter fixes
* Nav2, localization, and slam launch and config files
* Contributors: Roni Kreinin

0.1.2 (2022-09-15)
------------------
* ci: Fixed flake8 lint
* Contributors: Daisuke Nishimatsu, Roni Kreinin

0.1.1 (2022-07-12)
------------------
* Install turtlebot4_navigation module
* Added turtlebot4 navigator
* Fixed AMCL robot_model_type parameter
* Contributors: Roni Kreinin, roni-kreinin

0.1.0 (2022-05-03)
------------------
* First Galactic release
* Contributors: Roni Kreinin
