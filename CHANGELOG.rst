^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package systemd_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2020-11-04)
-------------------
* Updated documentation
* Python 2/3 compatibility

1.9.0 (2020-10-16)
------------------
* Python 3 compatibility

1.8.0 (2020-09-08)
------------------
* reduced infinite to 3h
* infinte start timeout, to allow warehouse and firmware updates, slightly increase stop timeout

1.7.0 (2020-08-26)
------------------
* Set all parameters on reload
  The old behavior was just setting parameters which changed between start
  and reload without checking what is really on the parameter server.
  The consequence so far was that for example after a call of
  "rosparam delete /" reload did not set any parameter.
  Closes: SW-34127

1.6.2 (2020-04-30)
------------------
* Add localhost IPs to filter (Closes: SW-33832)

1.6.1 (2020-04-06)
------------------
* disable rostime to integrate nicely into gazebo

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
