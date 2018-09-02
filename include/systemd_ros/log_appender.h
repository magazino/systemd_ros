#ifndef _SYSTEMD_LOGGING_APPENDER_H
#define _SYSTEMD_LOGGING_APPENDER_H

#include <log4cxx/appenderskeleton.h>

namespace log4cxx
{

class LOG4CXX_EXPORT SystemJournalAppender : public log4cxx::AppenderSkeleton
{
public:
  DECLARE_LOG4CXX_OBJECT(SystemJournalAppender)
  BEGIN_LOG4CXX_CAST_MAP()
    LOG4CXX_CAST_ENTRY(SystemJournalAppender)
    LOG4CXX_CAST_ENTRY_CHAIN(AppenderSkeleton)
  END_LOG4CXX_CAST_MAP()

  ~SystemJournalAppender();

protected:
  void append(const log4cxx::spi::LoggingEventPtr& event,
              log4cxx::helpers::Pool& p);

  virtual bool requiresLayout() const
  {
    return false;
  }

  virtual void close();
};

} // namespace log4cxx

#endif // _SYSTEMD_LOGGING_APPENDER_H
