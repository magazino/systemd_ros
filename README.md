# ROS on systemd

systemd is the first process started by Linux that monitors services, other
daemons, other processes, the process outputs and offers a variety of tools to
do that.

ROS instead has a couple of tools or systems in place that somehow try to
solve similar problems:

* roslaunch: Process monitor and launch infrastructure for multiple processes
* rosconsole / rosout: Library support for logging node output

This package tries to provide alternatives which have even more features than
the ROS equivalents.

## Logging

The de-facto standard in ROS for logging is the rosconsole/rosout system:

* ``rospy.loginfo`` and friends in PYTHON
* ``ROS_LOGINFO`` and similar macros for C++

For both Python and C++ these functions are based on standard logging libraries
(python logging or log4cxx). They implement custom logging handlers and e.g.
publish messages on the ROS topic ``/rosout``, colorize output or print
to the terminal with a custom formatter.

Nodes written in Python have persistent log files in ``.ros`` (roscd log) whereas
C++ nodes lack the feature of persistent log files in the default configuration.
Additionally there is another program call rosout which subscribes to the
``/rosout`` topic and provides the ``rosout.log`` file.
This program is started with the standard roscore
(see ``/opt/ros/noetic/etc/ros/roscore.xml``) and rotates log files in a
handcrafted way. Log rotation is not in place for Python or C++ nodes. For
the latter it's not even possible.

### journald

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

### Configuration

#### C++ with roscpp

Configuration is not needed but the forked version of roscpp and rosconsole:

 - https://github.com/magazino/rosconsole/tree/noetic-devel
 - https://github.com/magazino/ros_comm/tree/noetic-devel

#### Python with rospy

rospy allows to configure the logging using an ini file.

[robot-systemd](https://github.com/magazino/robot-systemd/) contains an example
configuration in ``/etc/ros/python_logging.conf`` which enables the journal
handler:

```ini
...

[handler_journalHandler]
class=systemd_ros.JournalHandler
level=DEBUG
formatter=journalFormatter
args=()

...
```

In addition to that the following export is needed so that rospy reads the
configuration. It is provided in ``/etc/profile.d/ros_python_logging.sh`` as
part of  [robot-systemd](https://github.com/magazino/robot-systemd/).

```bash
#!/bin/sh
export ROS_PYTHON_LOG_CONFIG_FILE="/etc/ros/python_logging.conf"
```

#### Logging of remote nodes

Roslaunch allows to start nodes remotely via SSH. Forwarding log messages of such
remote nodes into the local journal is only possible via the systemd-generator
but not with bare-metal roslaunch.

### Using journalctl

Internally the ROS Node name is used as ``SYSLOG_IDENTIFIER``. This allows
filtering for logs of individual nodes. More information can be found
[here](https://www.freedesktop.org/software/systemd/man/journalctl.html).

```bash
journalctl -f SYSLOG_IDENTIFIER=/talker
journalctl -S yesterday SYSLOG_IDENTIFIER=/talker
journalctl -S "2020-06-01 0:00:00" -U "2020-06-02 00:00:00"
journalctl -S yesterday -P err
```

## Launch Management

This section describes how systemd-generators can be used to automate the
process of translating launch files to systemd service files. The example is
provided as a proper Debian package in
[robot-systemd](https://github.com/magazino/robot-systemd/).


### systemd-generators

[Systemd generators](https://www.freedesktop.org/software/systemd/man/systemd.generator.html)
are small executables that live in ``/usr/lib/systemd/system-generators/`` and
other directories listed above. systemd will execute those binaries very early
at bootup and at configuration reload time — before unit files are loaded.
Their main purpose is to convert configuration that is not native into
dynamically generated unit files.

This mechanism is used to translate launch files into services files during 
boot time.

### Starting roscore as a service

systemd_ros contains a slightly modified roscore which makes use of
[socket activation](http://0pointer.de/blog/projects/socket-activation.html).

The files roscore.socket and roscore.service need to be installed (please have
a look at the [robot-systemd](https://github.com/magazino/robot-systemd/)
package.

This is a prerequisite for the generated services to work since they depend on
``roscore.service``.

### Example generator

Example generator which creates services files for user robot:robot to launch
``robot_bringup/launch/robot.launch``: ``/usr/lib/systemd/system-generators/robot-bringup``

```bash
#!/bin/sh -e

. /etc/ros/robot-bringup

export ROS_DISTRO=noetic
export LD_LIBRARY_PATH="/opt/ros/$ROS_DISTRO/lib"
export PYTHONPATH="/opt/ros/$ROS_DISTRO/lib/python3/dist-packages"
export ROS_PACKAGE_PATH="/opt/ros/$ROS_DISTRO/share"
export ROS_MASTER_URI="http://$HOSTNAME:11311"
export PATH="$PATH:/opt/ros/$ROS_DISTRO/lib/systemd_ros"
export ROS_ROOT="/opt/ros/$ROS_DISTRO/share/ros"
export ROS_VERSION=1
export ROS_ETC_DIR="/opt/ros/$ROS_DISTRO/etc/ros"

LAUNCH_FILE="/opt/ros/$ROS_DISTRO/share/$PACKAGE/launch/$LAUNCH_FILE"

generate-services "$SERVICE" "$LAUNCH_FILE" "$3" --user "$USER" --group "$GROUP" --extra-config="/etc/ros/robot-bringup-extras.yaml" > "/$3/$SERVICE-generator.out" 2>&1
mkdir -p "$3/multi-user.target.wants/"
ln -s "$3/$SERVICE.service" "$3/multi-user.target.wants/"
```

The generated service files can be found in ``/var/run/systemd/generator.late/``.
If anything goes wrong during the service generation phase all error messages
are logged into ``/var/run/systemd/generator.late/robot-bringup.out`` (journald
is not available at the time when generators are executed).

The generated services will be started as part of multi-user.target in this
example.

Services can be regenerated by:

* First stopping the relevant service: ``systemctl stop robot-bringup``
* Reload daemons: ``systemctl daemon-reload``
* Start the services again: ``systemctl start robot-bringup``

The service ``robot-bringup.service`` is used to load all parameters on the
parameter server. Once they are set, all other services are started.
The services are named according to the following pattern
``robot-bringup-<node_name>.service``. Each node can be restarted individually.
