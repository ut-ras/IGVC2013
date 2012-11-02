#ifndef _ALEXTA_LOGGER_
#define _ALEXTA_LOGGER_

#include <stdio.h>
#include <stdarg.h>
#include <memory.h>

#include <exception>

using namespace std;

namespace brown_ros
{

enum LogLevel
{
  ERROR = 1,
  WARNING = 2,
  LVL1 = 3,
  LVL2 = 4,
  LVL3 = 5,
  LVL4 = 6,
  LVL5 = 7
};


///
///Exception class for all the exception handling
///
class BrownALException:public exception
{
private:
  //Description of what's happened
  char m_szDescription[1024];
public:
  //Standard constructor for the exception
  BrownALException()
  {
    memset(m_szDescription, 0, sizeof(m_szDescription));
  }

  //Contstructor to create an exception with printf-like arguments
  BrownALException(const char * format, ...)
  {
    memset(m_szDescription, 0, sizeof(m_szDescription));
    va_list args;
    va_start(args, format);
    vsnprintf(m_szDescription, 1023, format, args);
    va_end(args);
  }

  //Destructor for the exception. Deletes the string message
  ~BrownALException() throw()
  {
  }

  //Set prihtf-like error message
  void SetMessage(const char * format, ...)
  {
    va_list args;
    va_start(args, format);
    vsnprintf(m_szDescription, 1023, format, args);
    va_end(args);
  }

  //Necessary for the exception derived from std::exception
  virtual const char* what() const throw()
  {
    return m_szDescription;
  }
}; //End of exception class



///
///Class to log messages 
///
class Logger
{
  static const int cbMaxMsgSize = 10240;
private:
  //Message buffer
  char m_szMessage[10240];
  LogLevel m_currentLevelStdout;
  LogLevel m_currentLevelStderr;

  void PrintMessage(LogLevel logLevel, FILE* filePrint, char* szMessage)
  {
    if (logLevel == ERROR)
      fprintf(filePrint, "ERROR: %s\n", m_szMessage);
    else if (logLevel == WARNING)
      fprintf(filePrint, "WARNING: %s\n", m_szMessage);
    else
      fprintf(filePrint, "%s\n", m_szMessage);
    fflush(filePrint);
  }


public:

  //Default constructor
  Logger()
  {
    memset(m_szMessage, 0, sizeof(m_szMessage));
    m_currentLevelStdout = LVL3;
    m_currentLevelStderr = WARNING;
  }

  Logger(LogLevel levelStdout, LogLevel levelStderr)
  {
    memset(m_szMessage, 0, sizeof(m_szMessage));
    m_currentLevelStdout = levelStdout;
    m_currentLevelStderr = levelStderr;
  }

  void SetLevelStdout(LogLevel levelStdout)
  {
    m_currentLevelStdout = levelStdout;
  }

  void SetLevelStderr(LogLevel levelStderr)
  {
    m_currentLevelStderr = levelStderr;
  }

  //Log the printf-style message
  void Log(bool bError, bool bWarning, const char * format, ...)
  {
    va_list args;
    va_start(args, format);
    vsnprintf(m_szMessage, cbMaxMsgSize-1, format, args);
    va_end(args);
    if (bError)
    {
      fprintf(stderr, "ERROR: %s\n", m_szMessage);
      fprintf(stdout, "ERROR: %s\n", m_szMessage);
    }
    else if (bWarning)
    {
      fprintf(stderr, "WARNING: %s\n", m_szMessage);
      fprintf(stdout, "WARNING: %s\n", m_szMessage);
    }else
    {
      fprintf(stdout, " %s\n", m_szMessage);
    }
    fflush(stdout);
    fflush(stderr);
  }

  //Log the printf-style message
  void Logl(LogLevel logLevel, const char * format, ...)
  {
    va_list args;
    va_start(args, format);
    vsnprintf(m_szMessage, cbMaxMsgSize-1, format, args);
    va_end(args);
    if (logLevel <= m_currentLevelStderr)
    {
      PrintMessage(logLevel, stderr, m_szMessage);
    }
    if (logLevel <= m_currentLevelStdout)
    {
//      PrintMessage(logLevel, stdout, m_szMessage);
      PrintMessage(logLevel, stdout, m_szMessage);
    }
  }
}; //End of Logger class

} //End of alexta namespace
#endif	//_ALEXTA_LOGGER_


