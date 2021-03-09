#ifndef __DLOVI_EXCEPTION_H
#define __DLOVI_EXCEPTION_H

#include <exception>
#include <string>

namespace dlovi{
  class Exception : public std::exception{
  public:
    // Constructors and Destructors
    Exception(const std::string & strMessage);
    virtual ~Exception() throw();

    // Public Methods
    virtual const char * what() const throw();
    virtual void raise();
    void tag(const std::string & strClass, const std::string & strFunction) throw ();
    void tag(const std::string & strFunction) throw ();

  protected:
    // Members
    std::string m_strMessage;
    mutable std::string m_strFormattedMessage;
    bool m_bTagged;
  };
}

#endif
