#include <systemd_ros/log_appender.h>

#include <pluginlib/class_list_macros.h>
#include <ros/this_node.h>

#define SD_JOURNAL_SUPPRESS_LOCATION
#include <systemd/sd-journal.h>

using namespace log4cxx;
using namespace log4cxx::helpers;

IMPLEMENT_LOG4CXX_OBJECT(SystemJournalAppender)

SystemJournalAppender::~SystemJournalAppender()
{
}

void SystemJournalAppender::append(const log4cxx::spi::LoggingEventPtr& event,
                                   log4cxx::helpers::Pool&)
{
  const log4cxx::spi::LocationInfo& location_info = event->getLocationInformation();
  ::sd_journal_send(
    "MESSAGE=%s", event->getMessage().c_str(),
    "PRIORITY=%i", event->getLevel()->getSyslogEquivalent(),
    "CODE_FILE=%s", location_info.getFileName(),
    "CODE_LINE=%i", location_info.getLineNumber(),
    "CODE_FUNC=%s", location_info.getMethodName().c_str(),
    "SYSLOG_IDENTIFIER=%s", ros::this_node::getName().c_str(),
    NULL);
}

void SystemJournalAppender::close()
{
}

PLUGINLIB_EXPORT_CLASS(log4cxx::SystemJournalAppender, log4cxx::AppenderSkeleton)
