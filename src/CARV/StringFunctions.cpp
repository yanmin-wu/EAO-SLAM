#ifndef __DLOVI_STRINGFUNCTIONS_CPP
#define __DLOVI_STRINGFUNCTIONS_CPP

#include "CARV/StringFunctions.h"
#include "CARV/Exception.h"
#include <sstream>

using namespace std;

namespace dlovi{
  namespace stringfunctions{
    std::vector<std::string> split(const std::string & strOriginal, const std::string & strDelimiter){
      vector<string> retVal;
			try{
				string::size_type nOriginalLength = strOriginal.length();
				string::size_type nDelimiterLength = strDelimiter.length();
				string::size_type nFirst = 0;
				string::size_type nSecond;

				while(nFirst < nOriginalLength){
					nSecond = strOriginal.find(strDelimiter, nFirst);
					if(nSecond == string::npos){
						retVal.push_back(strOriginal.substr(nFirst));
						return retVal;
					}
					retVal.push_back(strOriginal.substr(nFirst, nSecond - nFirst));
					nFirst = nSecond + nDelimiterLength;
				}

				return retVal;
			}
      catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("stringfunctions", "split"); ex2.raise();
			}
      return retVal;
		}

		std::string join(const std::vector<std::string> & vStrArray, const std::string & strDelimiter){
      stringstream ssRetVal;
			try{
				int nNumStrings = vStrArray.size();

				if(nNumStrings > 0)
					ssRetVal << vStrArray[0];
				for(int i = 1; i < (int)vStrArray.size(); i++)
					ssRetVal << strDelimiter << vStrArray[i];

				return ssRetVal.str();
			}
      catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("stringfunctions", "join"); ex2.raise();
        return ssRetVal.str();
			}
		}

    std::string trim(const std::string & strOriginal){
      try{
        int nFirstPos, nSecondPos;
        const int nLastPos = strOriginal.size() - 1;

        // find first non-whitespace position
        for(nFirstPos = 0; nFirstPos <= nLastPos && std::isspace((unsigned char)strOriginal[nFirstPos]); nFirstPos++);
        //find last non-whitespace position
        for(nSecondPos = nLastPos; nSecondPos >= 0 && std::isspace((unsigned char)strOriginal[nSecondPos]); nSecondPos--);

        if(nSecondPos >= nFirstPos) //if the string has non-whitespace characters then return the substring
          return strOriginal.substr(nFirstPos, nSecondPos - nFirstPos + 1);
        return std::string(); // otherwise return empty string
      }
      catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("stringfunctions", "trim"); ex2.raise();
        return std::string();
      }
    }
	}
}

#endif
