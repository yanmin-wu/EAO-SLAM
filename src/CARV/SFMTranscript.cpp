#ifndef __SFMTRANSCRIPT_CPP
#define __SFMTRANSCRIPT_CPP

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <set>
#include "CARV/SFMTranscript.h"
#include "CARV/StringFunctions.h"
#include "CARV/Exception.h"
#include "CARV/Matrix.h"

namespace dlovi{
    namespace compvis{

        using namespace dlovi::stringfunctions;

        // Constructors and Destructors

        SFMTranscript::SFMTranscript(){
            m_enumTranscriptType = TT_UNKNOWN;
            m_bValid = false;
            m_nNumEntries = 0;
        }

        SFMTranscript::~SFMTranscript(){
            // NOP
        }

        // Getters

        int SFMTranscript::numEntries() const{
            try{
                return m_nNumEntries;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "numEntries"); ex2.raise();
            }
        }

        SFMTranscript::TranscriptType SFMTranscript::getTranscriptType() const{
            try{
                return m_enumTranscriptType;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getTranscriptType"); ex2.raise();
            }
        }

        int SFMTranscript::numLines() const{
            try{
                return (int)m_arrStrLine.size();
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "numLines"); ex2.raise();
            }
        }

        std::string SFMTranscript::getLine(const int nLineIndex) const{
            try{
                return m_arrStrLine[nLineIndex];
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getLine"); ex2.raise();
            }
        }

        std::string SFMTranscript::getEntryText(const int nIndex) const{
            try{
                return m_arrEntryText[nIndex];
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryText"); ex2.raise();
            }
        }

        SFMTranscript::EntryType SFMTranscript::getEntryType(const int nIndex) const{
            try{
                return m_arrEntryType[nIndex];
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryType"); ex2.raise();
            }
        }

        const std::vector<dlovi::Matrix> & SFMTranscript::getEntryPoints(const int nIndex) const{
            try{
                return m_arrPoints[nIndex];
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryPoints"); ex2.raise();
            }
        }

        const std::vector<dlovi::Matrix> & SFMTranscript::getEntryCamCenters(const int nIndex) const{
            try{
                return m_arrCamCenters[nIndex];
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryCamCenters"); ex2.raise();
            }
        }

        const std::vector<std::vector<int> > & SFMTranscript::getEntryVisList(const int nIndex) const{
            try{
                return m_arrVisLists[nIndex];
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryVisList"); ex2.raise();
            }
        }

        SFMTranscript::EntryType SFMTranscript::getEntryType_Step() const{
            try{
                return m_enumStepEntryType;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryType_Step"); ex2.raise();
            }
        }

        const SFMTranscript::EntryData & SFMTranscript::getEntryData_Step() const{
            try{
                return m_objStepEntryData;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryData_Step"); ex2.raise();
            }
        }

        const std::vector<dlovi::Matrix> & SFMTranscript::getEntryPoints_Step() const{
            try{
                return m_arrStepPoints;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryPoints_Step"); ex2.raise();
            }
        }

        const std::vector<dlovi::Matrix> & SFMTranscript::getEntryCamCenters_Step() const{
            try{
                return m_arrStepCamCenters;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryCamCenters_Step"); ex2.raise();
            }
        }

        const std::vector<std::vector<int> > & SFMTranscript::getEntryVisList_Step() const{
            try{
                return m_arrStepVisLists;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryVisList_Step"); ex2.raise();
            }
        }

        const std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & SFMTranscript::getEntryClusters_Step() const{
            try{
                return m_arrStepClusters;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryClusters_Step"); ex2.raise();
            }
        }

        const std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & SFMTranscript::getEntryLines_Step() const{
            try{
                return m_arrStepLines;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryLines_Step"); ex2.raise();
            }
        }

        const std::vector<std::vector<int> > & SFMTranscript::getEntryClusterLineList_Step() const {
            try{
                return m_arrStepClusterLineList;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryClusterLineList_Step"); ex2.raise();
            }
        }

        const std::vector<int> & SFMTranscript::getEntryLineVisList_Step() const {
            try{
                return m_arrStepLineVisLists;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "getEntryLineVisList_Step"); ex2.raise();
            }
        }


        bool SFMTranscript::isIncrementalSFM() const{
            try{
                bool retVal;

                switch(getTranscriptType()){
                    case TT_PTAM:
                        retVal = true;
                        break;
                    default:
                        retVal = false;
                }

                return retVal;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "isIncrementalSFM"); ex2.raise();
            }
        }

        bool SFMTranscript::isValid() const{
            try{
                if( ! m_bValid || getTranscriptType() == TT_UNKNOWN)
                    return false;
                return true;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "isValid"); ex2.raise();
            }
        }

        // Setters

        // Public Methods

        void SFMTranscript::readFromFile(const std::string & strFileName){
            try{
                std::string strTmp;

                std::ifstream fileIn(strFileName.c_str(), std::ios::in);
                if(!fileIn)
                    throw dlovi::Exception("Could not open file");

                while(std::getline(fileIn, strTmp))
                    addLine(strTmp);

                fileIn.close();
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "readFromFile"); ex2.raise();
            }
        }

        void SFMTranscript::writeToFile(const std::string & strFileName) const{
            try{
                std::string strTmp;

                std::ofstream fileOut(strFileName.c_str(), std::ios::out);
                if(!fileOut)
                    throw dlovi::Exception("Could not open file");

                for(int i = 0; i < numLines(); i++)
                    fileOut << getLine(i) << "\n";
                fileOut.flush();
                fileOut.close();
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "writeToFile"); ex2.raise();
            }
        }

        void SFMTranscript::processTranscriptText(){
            try{
                int nLineIndex = parseTranscriptHeader();
                nLineIndex = parseTranscriptBody(nLineIndex);
                markAsValid();
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "processTranscriptText"); ex2.raise();
            }
        }

        void SFMTranscript::stepTranscriptText(bool bFirstEntry){
            try{
                static int nLineIndex = 0;

                if(bFirstEntry)
                    nLineIndex = parseTranscriptHeader();

//                nLineIndex = stepTranscriptBody(m_enumStepEntryType, m_objStepEntryData, m_arrStepPoints, m_arrStepCamCenters, m_arrStepVisLists, nLineIndex);
                nLineIndex = stepTranscriptBody(m_enumStepEntryType, m_objStepEntryData, m_arrStepPoints, m_arrStepCamCenters, m_arrStepVisLists, nLineIndex,
                                                m_arrStepClusters, m_arrStepLines, m_arrStepClusterLineList, m_arrStepLineVisLists);

                if(nLineIndex >= numLines())
                    markAsValid(); // we're done.
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "processTranscriptText"); ex2.raise();
            }
        }

        void SFMTranscript::addLine(const std::string & line){
            try{
                m_arrStrLine.push_back(line);
                invalidate();
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "addLine"); ex2.raise();
            }
        }

        void SFMTranscript::invalidate(){
            try{
                m_bValid = false;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "invalidate"); ex2.raise();
            }
        }

        // Private Methods

        void SFMTranscript::prepareForNewEntry(){
            try{
                m_nNumEntries++; // Increment the # of entries

                // Initialize new entry to empty
                m_arrEntryText.push_back("");
                m_arrEntryType.push_back(ET_INVALID);

                if(numEntries() >= 2){ // 2 and not 1, because m_nNumEntries was already incremented above
                    // Duplicate the last entry's set of points, cameras, and visibility lists for initialization of this entry.
                    m_arrPoints.push_back(m_arrPoints[numEntries() - 2]);
                    m_arrCamCenters.push_back(m_arrCamCenters[numEntries() - 2]);
                    m_arrVisLists.push_back(m_arrVisLists[numEntries() - 2]);
                }
                else{
                    // Add an empty set of points, cameras, and visibility lists for initialization of this entry.
                    m_arrPoints.push_back(std::vector<dlovi::Matrix>());
                    m_arrCamCenters.push_back(std::vector<dlovi::Matrix>());
                    m_arrVisLists.push_back(std::vector<std::vector<int> >());
                }
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "prepareForNewEntry"); ex2.raise();
            }
        }

        void SFMTranscript::prepareForNewEntry(EntryType enumEntryType){
            try{
                prepareForNewEntry(); // Does most the work
                m_arrEntryType[numEntries() - 1] = enumEntryType; // Sets the entry type
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "prepareForNewEntry"); ex2.raise();
            }
        }

        void SFMTranscript::prepareForNewEntry_Step(EntryType & enumEntryType, EntryType enumNewEntryType){
            try{
                m_nNumEntries++; // Increment the # of entries
                m_objStepEntryData.clear(); // Initialize the entry data to empty
                // Initialize new entry type
                enumEntryType = enumNewEntryType;
                // Note: The last entry's set of points, cameras, and visibility lists are duplicated for initialization of
                // this entry implicitly (ie: nothing done to them.)
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "prepareForNewEntry_Step"); ex2.raise();
            }
        }

        int SFMTranscript::parseTranscriptHeader(int nLoop){
            try{
                std::string strCurrentLine;
                bool bInHeader = true;

                for( ; nLoop < numLines() && bInHeader; nLoop++){
                    strCurrentLine = trim(getLine(nLoop));

                    if(strCurrentLine.empty())
                        continue;
                    else if(strCurrentLine == "*** BODY ***")
                        bInHeader = false;
                    else if(strCurrentLine.find("SFM Transcript: ") != std::string::npos){
                        std::vector<std::string> arrStr = split(strCurrentLine, " ");
                        if(arrStr[2] == "PTAM")
                            setTranscriptType(TT_PTAM);
                        else if(arrStr[2] == "ORBSLAM")
                            setTranscriptType(TT_ORBSLAM);
                        else{
                            setTranscriptType(TT_UNKNOWN);
                            throw dlovi::Exception("Transcript type not recognized.");
                        }
                    }
                    else{
                        setTranscriptType(TT_UNKNOWN);
                        throw dlovi::Exception("Unrecognized line in transcript header.");
                    }
                }

                if(getTranscriptType() == TT_UNKNOWN)
                    throw dlovi::Exception("Empty Header.");

                return nLoop;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "parseTranscriptHeader"); ex2.raise();
            }
        }

        int SFMTranscript::parseTranscriptBody(int nLoop){
            try{
                std::string strCurrentLine;
                bool bInBody = true;

                for( ; nLoop < numLines() && bInBody; nLoop++){
                    strCurrentLine = trim(getLine(nLoop));

                    if(strCurrentLine.empty())
                        continue;
//          else if(strCurrentLine == "*** END ***")
//            bInBody = false;
                    else if(strCurrentLine.find("reset") != std::string::npos){
                        // eg: reset
                        prepareForNewEntry(ET_RESET);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        m_arrPoints[numEntries() - 1].clear();
                        m_arrCamCenters[numEntries() - 1].clear();
                        m_arrVisLists[numEntries() - 1].clear();
                    }
                    else if(strCurrentLine.find("new point: ") != std::string::npos){
                        // eg: new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN // vis list optional
                        prepareForNewEntry(ET_POINTINSERTION);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;
                        if(strCurrentLine.find(", ") != std::string::npos){
                            // Visibility list is present
                            std::vector<std::string> arrStr = split(strCurrentLine.substr(11), ", "); // 11 == strlen("new point: ")

                            Matrix matNewPoint(arrStr[0]);
                            m_arrPoints[numEntries() - 1].push_back(matNewPoint);

                            for(int i = 1; i < (int)arrStr.size(); i++){
                                int nCamIndex = atoi(arrStr[i].c_str());
                                m_arrVisLists[numEntries() - 1][nCamIndex].push_back((int)m_arrPoints[numEntries() - 1].size() - 1);
                            }
                        }
                        else{
                            // Visibility list not present
                            Matrix matNewPoint(strCurrentLine.substr(11)); // 11 == strlen("new point: ")
                            m_arrPoints[numEntries() - 1].push_back(matNewPoint);
                        }
                    }
                    else if(strCurrentLine.find("del point: ") != std::string::npos){
                        // eg: del point: 17
                        prepareForNewEntry(ET_POINTDELETION);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        int nPointIndex = atoi(strCurrentLine.substr(11).c_str()); // 11 == strlen("del point: ")

                        // Instead of deleting the point from the points vector
                        // (which would need decrementing other point indicies and result in costly maintenance),
                        // leave a "ghost entry" and only erase all occurences of this index from the visibility lists.
                        std::vector<std::vector<int> >::iterator it;
                        for(it = m_arrVisLists[numEntries() - 1].begin(); it != m_arrVisLists[numEntries() - 1].end(); it++){
                            std::vector<int> vTmpVisList;
                            std::remove_copy(it->begin(), it->end(), std::back_inserter(vTmpVisList), nPointIndex);
                            *it = vTmpVisList;
                        }
                    }
                    else if(strCurrentLine.find("move point: ") != std::string::npos){
                        // eg: move point: 7, [x; y; z]
                        prepareForNewEntry(ET_POINTUPDATE);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        std::vector<std::string> arrStr = split(strCurrentLine.substr(12), ", "); // 12 == strlen("move point: ")
                        int nPointIndex = atoi(arrStr[0].c_str());
                        Matrix matNewPoint(arrStr[1]);
                        m_arrPoints[numEntries() - 1][nPointIndex] = matNewPoint;
                    }
                    else if(strCurrentLine.find("del observation: ") != std::string::npos){
                        // eg: del observation: camIndex, pointIndex
                        prepareForNewEntry(ET_VISIBILITYRAYDELETION);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        std::vector<std::string> arrStr = split(strCurrentLine, " ");
                        int nCamIndex = atoi(arrStr[2].c_str());
                        int nPointIndex = atoi(arrStr[3].c_str());
                        // Find and remove the observation from the visibility list
                        for(std::vector<int>::iterator it = m_arrVisLists[numEntries() - 1][nCamIndex].begin();
                            it != m_arrVisLists[numEntries() - 1][nCamIndex].end(); it++){
                            if(*it == nPointIndex){
                                m_arrVisLists[numEntries() - 1][nCamIndex].erase(it);
                                break;
                            }
                        }
                    }
                    else if(strCurrentLine.find("observation: ") != std::string::npos){
                        // eg: observation: camIndex, pointIndex
                        prepareForNewEntry(ET_VISIBILITYRAYINSERTION);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        std::vector<std::string> arrStr = split(strCurrentLine, " ");
                        int nCamIndex = atoi(arrStr[1].c_str());
                        int nPointIndex = atoi(arrStr[2].c_str());
                        m_arrVisLists[numEntries() - 1][nCamIndex].push_back(nPointIndex);
                    }
                    else if(strCurrentLine.find("new cam: ") != std::string::npos){
                        // eg:
                        // new cam: [x; y; z] {
                        // new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN // vis list NOT optional
                        // ...
                        // observation: pointIndex // Excludes new points.
                        // ...
                        // }
                        prepareForNewEntry(ET_KEYFRAMEINSERTION);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        Matrix matNewCam(trim(strCurrentLine.substr(0, strCurrentLine.length() - 1).substr(9))); // 9 == strlen("new cam: "), also remove the "{"
                        m_arrCamCenters[numEntries() - 1].push_back(matNewCam); // new camera center
                        m_arrVisLists[numEntries() - 1].push_back(std::vector<int>()); // empty visibility list, populated below

                        do{
                            if(++nLoop >= numLines())
                                throw dlovi::Exception("Transcript ended abruptly in new-camera block.");

                            strCurrentLine = trim(getLine(nLoop));
                            m_arrEntryText[numEntries() - 1] += "\n" + strCurrentLine;

                            if(strCurrentLine.find("new point: ") != std::string::npos){
                                // eg: new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN
                                std::vector<std::string> arrStr = split(strCurrentLine.substr(11), ", "); // 11 == strlen("new point: ")

                                Matrix matNewPoint(arrStr[0]);
                                m_arrPoints[numEntries() - 1].push_back(matNewPoint);

                                // Process this point's vis list (e.g.: initialized from epipolar search, etc, can = more than 1 KF)
                                for(int i = 1; i < (int)arrStr.size(); i++){
                                    int nCamIndex = atoi(arrStr[i].c_str());
                                    m_arrVisLists[numEntries() - 1][nCamIndex].push_back((int)m_arrPoints[numEntries() - 1].size() - 1);
                                }
                            }
                            else if(strCurrentLine.find("observation: ") != std::string::npos)
                            {
                                int nCamIndex = (int)m_arrCamCenters[numEntries() - 1].size() - 1; // the new camera
                                int nPointIndex = atoi(strCurrentLine.substr(13).c_str()); // 13 == strlen("observation: ")
                                m_arrVisLists[numEntries() - 1][nCamIndex].push_back(nPointIndex);
                            }
                            else if(strCurrentLine != "}")
                                throw dlovi::Exception("Unrecognized line in new-camera block.");
                        } while(strCurrentLine != "}");
                    }
//          else if(strCurrentLine.find("") != std::string::npos){
//            // eg: del cam: 17
//            prepareForNewEntry(ET_KEYFRAMEDELETE);
//            // TODO: stub (leave as stub for now, not utilized by PTAM)
//          }
                    else if(strCurrentLine.find("move cam: ") != std::string::npos){
                        // eg: move cam: 7, [x; y; z]
                        prepareForNewEntry(ET_KEYFRAMEUPDATE);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        std::vector<std::string> arrStr = split(strCurrentLine.substr(10), ", "); // 10 == strlen("move cam: ")
                        int nCamIndex = atoi(arrStr[0].c_str());
                        Matrix matNewCam(arrStr[1]);
                        m_arrCamCenters[numEntries() - 1][nCamIndex] = matNewCam;
                    }
                    else if(strCurrentLine.find("bundle {") != std::string::npos){
                        // eg:
                        // bundle {
                        // move point: 17, [x; y; z]
                        // ...
                        // move cam: 24, [x; y; z]
                        // ...
                        // }
                        prepareForNewEntry(ET_BUNDLEADJUSTMENT);
                        m_arrEntryText[numEntries() - 1] = strCurrentLine;

                        do{
                            if(++nLoop >= numLines())
                                throw dlovi::Exception("Transcript ended abruptly in bundle adjustment block.");

                            strCurrentLine = trim(getLine(nLoop));
                            m_arrEntryText[numEntries() - 1] += "\n" + strCurrentLine;

                            if(strCurrentLine.find("move point: ") != std::string::npos){
                                std::vector<std::string> arrStr = split(strCurrentLine.substr(12), ", "); // 12 == strlen("move point: ")
                                int nPointIndex = atoi(arrStr[0].c_str());
                                Matrix matNewPoint(arrStr[1]);
                                m_arrPoints[numEntries() - 1][nPointIndex] = matNewPoint;
                            }
                            else if(strCurrentLine.find("move cam: ") != std::string::npos)
                            {
                                std::vector<std::string> arrStr = split(strCurrentLine.substr(10), ", "); // 10 == strlen("move cam: ")
                                int nCamIndex = atoi(arrStr[0].c_str());
                                Matrix matNewCam(arrStr[1]);
                                m_arrCamCenters[numEntries() - 1][nCamIndex] = matNewCam;
                            }
                            else if(strCurrentLine != "}")
                                throw dlovi::Exception("Unrecognized line in bundle adjustment block.");
                        } while(strCurrentLine != "}");
                    }
                    else
                        throw dlovi::Exception("Unrecognized line in transcript body.");
                }

                bInBody = false;

                // DEBUG INFO:
                //cerr << "-------------------" << endl;
                //for(int feh = 0; feh < numEntries(); feh++)
                //  cerr << getEntryText(feh) << endl << endl;
                //cerr << "-------------------" << endl;

                return nLoop;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "parseTranscriptBody"); ex2.raise();
            }
        }

        int SFMTranscript::stepTranscriptBody(EntryType & enumEntryType, EntryData & objEntryData, std::vector<dlovi::Matrix> & arrPoints,
                                              std::vector<dlovi::Matrix> & arrCamCenters, std::vector<std::vector<int> > & arrVisLists, int nLoop){

            // TODO: Remove me
            //static std::set<int> setDeletedPoints;
            //static std::set<int> setReaddedPoints;
            // -------------------

            try{
                std::string strCurrentLine = trim(getLine(nLoop));
//                std::cout << strCurrentLine << endl;

                // TODO: throw exception if we go past the # of lines
                while(strCurrentLine.empty())
                    strCurrentLine = trim(getLine(++nLoop));

                if(strCurrentLine.find("reset") != std::string::npos){
                    // eg: reset
                    prepareForNewEntry_Step(enumEntryType, ET_RESET);

                    arrPoints.clear();
                    arrCamCenters.clear();
                    arrVisLists.clear();
                }
                else if(strCurrentLine.find("new point: ") != std::string::npos){
                    // eg: new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN // vis list optional
                    prepareForNewEntry_Step(enumEntryType, ET_POINTINSERTION);
                    if(strCurrentLine.find(", ") != std::string::npos){
                        // Visibility list is present
                        std::vector<std::string> arrStr = split(strCurrentLine.substr(11), ", "); // 11 == strlen("new point: ")

                        Matrix matNewPoint(arrStr[0]);
                        arrPoints.push_back(matNewPoint);
                        m_objStepEntryData.nPointIndex = (int)arrPoints.size() - 1;

                        for(int i = 1; i < (int)arrStr.size(); i++){
                            int nCamIndex = atoi(arrStr[i].c_str());
                            arrVisLists[nCamIndex].push_back((int)arrPoints.size() - 1);
                        }
                    }
                    else{
                        // Visibility list not present
                        Matrix matNewPoint(strCurrentLine.substr(11)); // 11 == strlen("new point: ")
                        arrPoints.push_back(matNewPoint);
                        m_objStepEntryData.nPointIndex = (int)arrPoints.size() - 1;
                    }
                }
                else if(strCurrentLine.find("del point: ") != std::string::npos){
                    // eg: del point: 17
                    prepareForNewEntry_Step(enumEntryType, ET_POINTDELETION);

                    int nPointIndex = atoi(strCurrentLine.substr(11).c_str()); // 11 == strlen("del point: ")
                    m_objStepEntryData.nPointIndex = nPointIndex;

                    // Instead of deleting the point from the points vector
                    // (which would need decrementing other point indicies and result in costly maintenance),
                    // leave a "ghost entry" and only erase all occurences of this index from the visibility lists.
                    std::vector<std::vector<int> >::iterator it;
                    for(it = arrVisLists.begin(); it != arrVisLists.end(); it++){
                        std::vector<int> vTmpVisList;
                        std::remove_copy(it->begin(), it->end(), std::back_inserter(vTmpVisList), nPointIndex);
                        *it = vTmpVisList;
                    }

                    // TODO: Remove me
                    //if(setDeletedPoints.count(nPointIndex) > 0)
                    //  cerr << "FJIALFJKAWLKJF" << endl;
                    //setDeletedPoints.insert(nPointIndex);
                    // ------------------
                }
                else if(strCurrentLine.find("move point: ") != std::string::npos){
                    // eg: move point: 7, [x; y; z]
                    prepareForNewEntry_Step(enumEntryType, ET_POINTUPDATE);

                    std::vector<std::string> arrStr = split(strCurrentLine.substr(12), ", "); // 12 == strlen("move point: ")
                    int nPointIndex = atoi(arrStr[0].c_str());
                    m_objStepEntryData.nPointIndex = nPointIndex;
                    Matrix matNewPoint(arrStr[1]);
                    arrPoints[nPointIndex] = matNewPoint;
                }
                else if(strCurrentLine.find("del observation: ") != std::string::npos){
                    // eg: del observation: camIndex, pointIndex
                    prepareForNewEntry_Step(enumEntryType, ET_VISIBILITYRAYDELETION);

                    std::vector<std::string> arrStr = split(strCurrentLine, " ");
                    int nCamIndex = atoi(arrStr[2].c_str());
                    int nPointIndex = atoi(arrStr[3].c_str());
                    m_objStepEntryData.nCamIndex = nCamIndex;
                    m_objStepEntryData.nPointIndex = nPointIndex;
                    // Find and remove the observation from the visibility list
                    for(std::vector<int>::iterator it = arrVisLists[nCamIndex].begin();
                        it != arrVisLists[nCamIndex].end(); it++){
                        if(*it == nPointIndex){
                            arrVisLists[nCamIndex].erase(it);
                            break;
                        }
                    }
                }
                else if(strCurrentLine.find("observation: ") != std::string::npos){
                    // eg: observation: camIndex, pointIndex
                    prepareForNewEntry_Step(enumEntryType, ET_VISIBILITYRAYINSERTION);

                    std::vector<std::string> arrStr = split(strCurrentLine, " ");
                    int nCamIndex = atoi(arrStr[1].c_str());
                    int nPointIndex = atoi(arrStr[2].c_str());
                    m_objStepEntryData.nCamIndex = nCamIndex;
                    m_objStepEntryData.nPointIndex = nPointIndex;
                    arrVisLists[nCamIndex].push_back(nPointIndex);

                    // TODO: Remove me
                    //if(setDeletedPoints.count(nPointIndex) > 0)
                    //  setReaddedPoints.insert(nPointIndex);
                    // ------------------
                }
                else if(strCurrentLine.find("new cam: ") != std::string::npos){
                    // eg:
                    // new cam: [x; y; z] {
                    // new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN // vis list NOT optional
                    // ...
                    // observation: pointIndex // Excludes new points.
                    // ...
                    // }
                    prepareForNewEntry_Step(enumEntryType, ET_KEYFRAMEINSERTION);

                    Matrix matNewCam(trim(strCurrentLine.substr(0, strCurrentLine.length() - 1).substr(9))); // 9 == strlen("new cam: "), also remove the "{"
                    arrCamCenters.push_back(matNewCam); // new camera center
                    arrVisLists.push_back(std::vector<int>()); // empty visibility list, populated below

                    m_objStepEntryData.nCamIndex = (int)arrCamCenters.size() - 1;

                    do{
                        if(nLoop++ >= numLines())
                            throw dlovi::Exception("Transcript ended abruptly in new-camera block.");

                        strCurrentLine = trim(getLine(nLoop));

                        if(strCurrentLine.find("new point: ") != std::string::npos){
                            // eg: new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN
                            std::vector<std::string> arrStr = split(strCurrentLine.substr(11), ", "); // 11 == strlen("new point: ")

                            Matrix matNewPoint(arrStr[0]);
                            arrPoints.push_back(matNewPoint);
                            m_objStepEntryData.arrPointIndices.push_back((int)arrPoints.size() - 1);

                            // Process this point's vis list (e.g.: initialized from epipolar search, etc, can = more than 1 KF)
                            for(int i = 1; i < (int)arrStr.size(); i++){
                                int nCamIndex = atoi(arrStr[i].c_str());
                                arrVisLists[nCamIndex].push_back((int)arrPoints.size() - 1);
                            }
                        }
                        else if(strCurrentLine.find("observation: ") != std::string::npos)
                        {
                            int nCamIndex = (int)arrCamCenters.size() - 1; // the new camera
                            int nPointIndex = atoi(strCurrentLine.substr(13).c_str()); // 13 == strlen("observation: ")
                            m_objStepEntryData.arrPointIndices.push_back(nPointIndex);
                            arrVisLists[nCamIndex].push_back(nPointIndex);
                        }
                        else if(strCurrentLine != "}")
                            throw dlovi::Exception("Unrecognized line in new-camera block.");
                    } while(strCurrentLine != "}");
                }
                else if(strCurrentLine.find("move cam: ") != std::string::npos){
                    // eg: move cam: 7, [x; y; z]
                    prepareForNewEntry_Step(enumEntryType, ET_KEYFRAMEUPDATE);

                    std::vector<std::string> arrStr = split(strCurrentLine.substr(10), ", "); // 10 == strlen("move cam: ")
                    int nCamIndex = atoi(arrStr[0].c_str());
                    m_objStepEntryData.nCamIndex = nCamIndex;
                    Matrix matNewCam(arrStr[1]);
                    arrCamCenters[nCamIndex] = matNewCam;
                }
                else if(strCurrentLine.find("bundle {") != std::string::npos){
                    // eg:
                    // bundle {
                    // move point: 17, [x; y; z]
                    // ...
                    // move cam: 24, [x; y; z]
                    // ...
                    // }
                    prepareForNewEntry_Step(enumEntryType, ET_BUNDLEADJUSTMENT);

                    do{
                        if(++nLoop >= numLines())
                            throw dlovi::Exception("Transcript ended abruptly in bundle adjustment block.");

                        strCurrentLine = trim(getLine(nLoop));

                        if(strCurrentLine.find("move point: ") != std::string::npos){
                            std::vector<std::string> arrStr = split(strCurrentLine.substr(12), ", "); // 12 == strlen("move point: ")
                            int nPointIndex = atoi(arrStr[0].c_str());
                            m_objStepEntryData.arrPointIndices.push_back(nPointIndex);
                            Matrix matNewPoint(arrStr[1]);
                            arrPoints[nPointIndex] = matNewPoint;
                        }
                        else if(strCurrentLine.find("move cam: ") != std::string::npos){
                            std::vector<std::string> arrStr = split(strCurrentLine.substr(10), ", "); // 10 == strlen("move cam: ")
                            int nCamIndex = atoi(arrStr[0].c_str());
                            m_objStepEntryData.arrCamIndices.push_back(nCamIndex);
                            Matrix matNewCam(arrStr[1]);
                            arrCamCenters[nCamIndex] = matNewCam;
                        }
                        else if(strCurrentLine != "}")
                            throw dlovi::Exception("Unrecognized line in bundle adjustment block.");
                    } while(strCurrentLine != "}");
                }
                else
                    throw dlovi::Exception("Unrecognized line in transcript body.");

                // TODO: Remove me
                //if(nLoop + 1 >= numLines()){
                // Then we're at the end of the transcript, so:
                //  std::cerr << "DEBUG INFO: Final # of re-added points: " << setReaddedPoints.size() << std::endl;
                //}
                // ------------------

                return nLoop + 1;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "stepTranscriptBody"); ex2.raise();
            }
        }

        int SFMTranscript::stepTranscriptBody(EntryType & enumEntryType, EntryData & objEntryData, std::vector<dlovi::Matrix> & arrPoints,
                                              std::vector<dlovi::Matrix> & arrCamCenters, std::vector<std::vector<int> > & arrVisLists, int nLoop,
                                              std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & arrClusters,
                                              std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & arrLines,
                                              std::vector<std::vector<int> > & arrClusterLineList,
                                              std::vector<int> & arrLineVisLists ){

            try{
                std::string strCurrentLine = trim(getLine(nLoop));

                // TODO: throw exception if we go past the # of lines
                while(strCurrentLine.empty())
                    strCurrentLine = trim(getLine(++nLoop));

                if(strCurrentLine.find("reset") != std::string::npos){
                    // eg: reset
                    prepareForNewEntry_Step(enumEntryType, ET_RESET);

                    arrPoints.clear();
                    arrCamCenters.clear();
                    arrVisLists.clear();
                    arrLineVisLists.clear();
                }
                else if(strCurrentLine.find("new point: ") != std::string::npos){
                    // eg: new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN // vis list optional
                    prepareForNewEntry_Step(enumEntryType, ET_POINTINSERTION);
                    if(strCurrentLine.find(", ") != std::string::npos){
                        // Visibility list is present
                        std::vector<std::string> arrStr = split(strCurrentLine.substr(11), ", "); // 11 == strlen("new point: ")

                        Matrix matNewPoint(arrStr[0]);
                        arrPoints.push_back(matNewPoint);
                        m_objStepEntryData.nPointIndex = (int)arrPoints.size() - 1;

                        for(int i = 1; i < (int)arrStr.size(); i++){
                            int nCamIndex = atoi(arrStr[i].c_str());
                            arrVisLists[nCamIndex].push_back((int)arrPoints.size() - 1);
                        }
                    }
                    else{
                        // Visibility list not present
                        Matrix matNewPoint(strCurrentLine.substr(11)); // 11 == strlen("new point: ")
                        arrPoints.push_back(matNewPoint);
                        m_objStepEntryData.nPointIndex = (int)arrPoints.size() - 1;
                    }
                }
                else if(strCurrentLine.find("del point: ") != std::string::npos){
                    // eg: del point: 17
                    prepareForNewEntry_Step(enumEntryType, ET_POINTDELETION);

                    int nPointIndex = atoi(strCurrentLine.substr(11).c_str()); // 11 == strlen("del point: ")
                    m_objStepEntryData.nPointIndex = nPointIndex;

                    // Instead of deleting the point from the points vector
                    // (which would need decrementing other point indicies and result in costly maintenance),
                    // leave a "ghost entry" and only erase all occurences of this index from the visibility lists.
                    std::vector<std::vector<int> >::iterator it;
                    for(it = arrVisLists.begin(); it != arrVisLists.end(); it++){
                        std::vector<int> vTmpVisList;
                        std::remove_copy(it->begin(), it->end(), std::back_inserter(vTmpVisList), nPointIndex);
                        *it = vTmpVisList;
                    }
                }
                else if(strCurrentLine.find("move point: ") != std::string::npos){
                    // eg: move point: 7, [x; y; z]
                    prepareForNewEntry_Step(enumEntryType, ET_POINTUPDATE);

                    std::vector<std::string> arrStr = split(strCurrentLine.substr(12), ", "); // 12 == strlen("move point: ")
                    int nPointIndex = atoi(arrStr[0].c_str());
                    m_objStepEntryData.nPointIndex = nPointIndex;
                    Matrix matNewPoint(arrStr[1]);
                    arrPoints[nPointIndex] = matNewPoint;
                }
                else if(strCurrentLine.find("del observation: ") != std::string::npos){
                    // eg: del observation: camIndex, pointIndex
                    prepareForNewEntry_Step(enumEntryType, ET_VISIBILITYRAYDELETION);

                    std::vector<std::string> arrStr = split(strCurrentLine, " ");
                    int nCamIndex = atoi(arrStr[2].c_str());
                    int nPointIndex = atoi(arrStr[3].c_str());
                    m_objStepEntryData.nCamIndex = nCamIndex;
                    m_objStepEntryData.nPointIndex = nPointIndex;
                    // Find and remove the observation from the visibility list
                    for(std::vector<int>::iterator it = arrVisLists[nCamIndex].begin();
                        it != arrVisLists[nCamIndex].end(); it++){
                        if(*it == nPointIndex){
                            arrVisLists[nCamIndex].erase(it);
                            break;
                        }
                    }
                }
                else if(strCurrentLine.find("observation: ") != std::string::npos){
                    // eg: observation: camIndex, pointIndex
                    prepareForNewEntry_Step(enumEntryType, ET_VISIBILITYRAYINSERTION);

                    std::vector<std::string> arrStr = split(strCurrentLine, " ");
                    int nCamIndex = atoi(arrStr[1].c_str());
                    int nPointIndex = atoi(arrStr[2].c_str());
                    m_objStepEntryData.nCamIndex = nCamIndex;
                    m_objStepEntryData.nPointIndex = nPointIndex;
                    arrVisLists[nCamIndex].push_back(nPointIndex);
                }
                else if(strCurrentLine.find("new cam: ") != std::string::npos){
                    // eg:
                    // new cam: [x; y; z] {
                    // new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN // vis list NOT optional
                    // ...
                    // observation: pointIndex // Excludes new points.
                    // ...
                    // }
                    prepareForNewEntry_Step(enumEntryType, ET_KEYFRAMEINSERTION);

                    Matrix matNewCam(trim(strCurrentLine.substr(0, strCurrentLine.length() - 1).substr(9))); // 9 == strlen("new cam: "), also remove the "{"
                    arrCamCenters.push_back(matNewCam); // new camera center
                    arrVisLists.push_back(std::vector<int>()); // empty visibility list, populated below

                    m_objStepEntryData.nCamIndex = (int)arrCamCenters.size() - 1;

                    do{
                        if(nLoop++ >= numLines())
                            throw dlovi::Exception("Transcript ended abruptly in new-camera block.");

                        strCurrentLine = trim(getLine(nLoop));

                        if(strCurrentLine.find("new point: ") != std::string::npos){
                            // eg: new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN
                            std::vector<std::string> arrStr = split(strCurrentLine.substr(11), ", "); // 11 == strlen("new point: ")

                            Matrix matNewPoint(arrStr[0]);
                            arrPoints.push_back(matNewPoint);
                            m_objStepEntryData.arrPointIndices.push_back((int)arrPoints.size() - 1);

                            // Process this point's vis list (e.g.: initialized from epipolar search, etc, can = more than 1 KF)
                            for(int i = 1; i < (int)arrStr.size(); i++){
                                int nCamIndex = atoi(arrStr[i].c_str());
                                arrVisLists[nCamIndex].push_back((int)arrPoints.size() - 1);
                            }
                        }
                        else if(strCurrentLine.find("observation: ") != std::string::npos)
                        {
                            int nCamIndex = (int)arrCamCenters.size() - 1; // the new camera
                            int nPointIndex = atoi(strCurrentLine.substr(13).c_str()); // 13 == strlen("observation: ")
                            m_objStepEntryData.arrPointIndices.push_back(nPointIndex);
                            arrVisLists[nCamIndex].push_back(nPointIndex);
                        }
                        else if(strCurrentLine != "}")
                            throw dlovi::Exception("Unrecognized line in new-camera block.");
                    } while(strCurrentLine != "}");
                }
                else if(strCurrentLine.find("move cam: ") != std::string::npos){
                    // eg: move cam: 7, [x; y; z]
                    prepareForNewEntry_Step(enumEntryType, ET_KEYFRAMEUPDATE);

                    std::vector<std::string> arrStr = split(strCurrentLine.substr(10), ", "); // 10 == strlen("move cam: ")
                    int nCamIndex = atoi(arrStr[0].c_str());
                    m_objStepEntryData.nCamIndex = nCamIndex;
                    Matrix matNewCam(arrStr[1]);
                    arrCamCenters[nCamIndex] = matNewCam;
                }
                else if(strCurrentLine.find("bundle {") != std::string::npos){
                    // eg:
                    // bundle {
                    // move point: 17, [x; y; z]
                    // ...
                    // move cam: 24, [x; y; z]
                    // ...
                    // }
                    prepareForNewEntry_Step(enumEntryType, ET_BUNDLEADJUSTMENT);

                    do{
                        if(++nLoop >= numLines())
                            throw dlovi::Exception("Transcript ended abruptly in bundle adjustment block.");

                        strCurrentLine = trim(getLine(nLoop));

                        if(strCurrentLine.find("move point: ") != std::string::npos){
                            std::vector<std::string> arrStr = split(strCurrentLine.substr(12), ", "); // 12 == strlen("move point: ")
                            int nPointIndex = atoi(arrStr[0].c_str());
                            m_objStepEntryData.arrPointIndices.push_back(nPointIndex);
                            Matrix matNewPoint(arrStr[1]);
                            arrPoints[nPointIndex] = matNewPoint;
                        }
                        else if(strCurrentLine.find("move cam: ") != std::string::npos){
                            std::vector<std::string> arrStr = split(strCurrentLine.substr(10), ", "); // 10 == strlen("move cam: ")
                            int nCamIndex = atoi(arrStr[0].c_str());
                            m_objStepEntryData.arrCamIndices.push_back(nCamIndex);
                            Matrix matNewCam(arrStr[1]);
                            arrCamCenters[nCamIndex] = matNewCam;
                        }
                        else if(strCurrentLine != "}")
                            throw dlovi::Exception("Unrecognized line in bundle adjustment block.");
                    } while(strCurrentLine != "}");
                }
                else if(strCurrentLine.find("new line: ") != std::string::npos) {
                    // eg: new line: [x; y; z], [x; y; z], Cluster_ind, [x; y; z], [x; y; z], KF_ind

                    prepareForNewEntry_Step(enumEntryType, ET_LINEINSERTION);

                    std::vector<std::string> arrStr = split(strCurrentLine.substr(10),
                                                            ", "); // 10 == strlen("new line: ")

                    Matrix matStartPoint(arrStr[0]);
                    Matrix matEndPoint(arrStr[1]);

                    int nClusterIndex = atoi(arrStr[2].c_str());

                    Matrix matClusterStartPoint(arrStr[3]);
                    Matrix matClusterEndPoint(arrStr[4]);

                    int nCamIndex = atoi(arrStr[5].c_str());

                    if (nClusterIndex < (int)arrClusters.size()) {
                        arrClusters[nClusterIndex].first = matClusterStartPoint;
                        arrClusters[nClusterIndex].second = matClusterEndPoint;
                    } else if (nClusterIndex == (int)arrClusters.size()) {
                        arrClusters.push_back(make_pair(matClusterStartPoint, matClusterEndPoint));
                        arrClusterLineList.push_back(std::vector<int>());
                    } else {
                        throw dlovi::Exception("Cluster index too large!");
                    }

                    if(nCamIndex >= (int)arrCamCenters.size()){
                        throw dlovi::Exception("Keyframe index too large!");
                    }

                    int nLineIndex = (int)arrLines.size();
                    arrLines.push_back(make_pair(matStartPoint, matEndPoint));

                    arrLineVisLists.push_back(nCamIndex);

                    arrClusterLineList[nClusterIndex].push_back(nLineIndex);

                    m_objStepEntryData.nClusterIndex = nClusterIndex;
                    m_objStepEntryData.nCamIndex = nCamIndex;
                    m_objStepEntryData.nLineIndex = nLineIndex;

                }
                else
                    throw dlovi::Exception("Unrecognized line in transcript body.");

                return nLoop + 1;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "stepTranscriptBody"); ex2.raise();
            }
        }

        void SFMTranscript::markAsValid(){
            try{
                m_bValid = true;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "markAsValid"); ex2.raise();
            }
        }

        // Private Setters

        void SFMTranscript::setTranscriptType(TranscriptType enumTranscriptType){
            try{
                m_enumTranscriptType = enumTranscriptType;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "setTranscriptType"); ex2.raise();
            }
        }
    }
}

#endif
