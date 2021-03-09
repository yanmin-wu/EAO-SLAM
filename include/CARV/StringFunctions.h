#ifndef __DLOVI_STRINGFUNCTIONS_H
#define __DLOVI_STRINGFUNCTIONS_H

#include <string>
#include <vector>
#include <cctype>

namespace dlovi{
  namespace stringfunctions{
		std::vector<std::string> split(const std::string & strOriginal, const std::string & strDelimiter);
		std::string join(const std::vector<std::string> & vStrArray, const std::string & strDelimiter);
    std::string trim(const std::string & strOriginal);
	}
}

#endif
