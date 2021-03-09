#ifndef __DLOVI_EXCEPTION_CPP
#define __DLOVI_EXCEPTION_CPP

#include "CARV/Exception.h"
#include <iostream>
#include <sstream>

namespace dlovi{

  // Constructors and Destructors

  Exception::Exception(const std::string & strMessage)
    : exception(), m_strMessage(strMessage), m_bTagged(false) { if(m_strMessage.empty()) m_strMessage = "no message"; }

  Exception::~Exception() throw() { }

  // Public Methods:

  const char * Exception::what() const throw(){
    try{
      std::stringstream ssMessage;

      if(m_bTagged)
        ssMessage << "Exception in " << m_strMessage;
      else
        ssMessage << "Exception: " << m_strMessage;

      m_strFormattedMessage = ssMessage.str();
      return m_strFormattedMessage.c_str();
    }
    catch(std::exception & e){
      std::cerr << "Warning in: dlovi::Exception::what(): Failed to format message. (msg = " << m_strMessage << ")" << std::endl;
      return "Exception: error formatting exception text";
    }
  }

  void Exception::raise(){
    throw *this;
  }

  void Exception::tag(const std::string & strClass, const std::string & strFunction) throw (){
    try{
      std::stringstream ssMessage;
      ssMessage << strClass << "::" << strFunction << "()" << (m_bTagged ? " -> " : ": ") << m_strMessage;
      m_strMessage = ssMessage.str();
      m_bTagged = true;
    }
    catch(std::exception & e){
      std::cerr << "Warning in: dlovi::Exception::tag(): Failed to tag.  (class = " << strClass << ", func = " << strFunction << ")"  << std::endl;
    }
  }

  void Exception::tag(const std::string & strFunction) throw (){
    try{
      std::stringstream ssMessage;
      ssMessage << strFunction << "()" << (m_bTagged ? " -> " : ": ") << m_strMessage;
      m_strMessage = ssMessage.str();
      m_bTagged = true;
    }
    catch(std::exception & e){
      std::cerr << "Warning in: dlovi::Exception::tag(): Failed to tag.  (func = " << strFunction << ")"  << std::endl;
    }
  }

}

#endif
