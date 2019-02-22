# ROS on systemd

## Motivation

Systemd is the first process started by Linux that monitors services, other
deamons, other processes, the process outputs and offers a variety of tools to
do that.

ROS instead has a couple of tools or systems in place that somehow try to
solve similar problems:

* roslaunch: Process monitor and launch infrastructure for multiple processes
* rosconsole / rosout: Library support for logging node output

While systemd is a well mentained software, roslaunch and other ros core
components maintained by OSRF lack development resources. Even pull requests
which add new features or fix bugs in the current system are often not
integrated into the core ROS packages.

Instead of trying to fix bugs in ill-maintained software this package
tries to provide alternatives which have even more features than the ROS
equivalents.

### Logging

The de-facto standard in ROS for logging is the rosconsole/rosout system:

* ``rospy.loginfo`` and friends in PYTHON
* ``ROS_LOGINFO`` and similar macros for C++

For both Python and C++ these functions are based on standard logging libraries
(python logging or log4cxx). They implement custom logging handlers and e.g.
publish messages on the ROS topic ``/rosout``, colorize output or print
to the terminal with a custom formatter.

Nodes written in Python have persistent logfiles in .ros (roscd log) whereas
C++ nodes lack the feature of persistent log files in the default configuration.
Additionally there is another program call rosout which subscribes to the
``/rosout`` topic and provides the ``rosout.log`` file.
This program is started with the standard roscore
(see ``/opt/ros/kinetic/etc/ros/roscore.xml``) and rotates log files in a
handcrafted way. Log rotation is not in place for Python or C++ nodes. For
the latter it's not even possible.

*Introducing journald:*

Journald is a system service for collecting and storing log data, introduced
with systemd. It tries to make it easier for system administrators to find
interesting and relevant information among an ever-increasing amount of log
messages. This feature is especially interesting for robot applications.

One of the main changes in journald compared to older systems
simple plain text log files with a special file format optimized for log
messages. This file format allows system administrators to access relevant
messages more efficiently. It also brings some of the power of database-driven
centralized logging implementations to individual systems.

The components introduced in this package / our forks of roscpp allow to
use the journald as a log sink for rosout logging messages. Rospy logging can
be configured using an ini file (``$ROS_PYTHON_LOG_CONFIG_FILE``).
