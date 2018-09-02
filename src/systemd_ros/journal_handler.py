from systemd import journal

import rospy


class JournalHandler(journal.JournalHandler):
    """Custom JournalHandler to add the ROS node name as SYSLOG_IDENTIFIER"""

    def emit(self, record):
        node_name = rospy.get_name()
        if node_name != rospy.get_namespace() + 'unnamed':
            self._extra['SYSLOG_IDENTIFIER'] = node_name
            # we are done patching this function
            self.emit = super(JournalHandler, self).emit
        super(JournalHandler, self).emit(record)
