^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package systemd_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2020-02-14)
------------------
* Make unit binding even stronger
  PartOf is limited to stopping and restarting of units. This allows to
  start a unit without roscore running. BindsTo should automatically start
  the roscore in that case.

1.5.0 (2020-01-14)
------------------
* Start main node after roscore

1.4.0 (2019-10-18)
------------------
* Define TimeoutStopSec

1.3.2 (2019-10-14)
------------------
* Fixed wants generation
* Support ros args

1.3.1 (2019-05-29)
------------------
* Fix typo in argument name

1.3.0 (2019-05-29)
------------------
* Services should be restarted if roscore restarts

1.2.0 (2019-05-07)
------------------
* Smart way of handling nodelets and managers

1.1.1 (2019-03-06)
------------------
* skip param loading during generation

1.1.0 (2019-03-05)
------------------
* Added service generator
* Removed C++ log appender
* Added README and License file

1.0.1 (2018-10-25)
------------------
* Fixed formatter for journald handler when used with rosout

1.0.0 (2018-09-12)
------------------
* Initial Release
