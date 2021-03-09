#ifndef __SFMTRANSCRIPT_H
#define __SFMTRANSCRIPT_H

#include <vector>
#include <string>

namespace dlovi{

    class Matrix;

    namespace compvis{

        class SFMTranscript;

        class SFMTranscript{
        public:
            // Enums
            enum TranscriptType {
                TT_UNKNOWN = -1,
                TT_PTAM,
                TT_ORBSLAM };
            enum EntryType {
                ET_INVALID = -1,
                ET_RESET,
                ET_POINTINSERTION,
                ET_POINTDELETION,
                ET_POINTUPDATE,
                ET_VISIBILITYRAYINSERTION,
                ET_VISIBILITYRAYDELETION,
                ET_KEYFRAMEINSERTION,
                ET_KEYFRAMEDELETE,
                ET_KEYFRAMEUPDATE,
                ET_BUNDLEADJUSTMENT,
                ET_LINEINSERTION };

            // Public Types
            class EntryData{
            public:
                // Constructors
                EntryData() : nPointIndex(-1), nCamIndex(-1) {}
                EntryData(const EntryData & ref){
                    nPointIndex = ref.nPointIndex; nCamIndex = ref.nCamIndex; arrPointIndices = ref.arrPointIndices; arrCamIndices = ref.arrCamIndices;
                }
                // Operators
                EntryData & operator=(const EntryData & ref){
                    if(this != & ref){
                        nPointIndex = ref.nPointIndex; nCamIndex = ref.nCamIndex; arrPointIndices = ref.arrPointIndices; arrCamIndices = ref.arrCamIndices;
                    }
                    return *this;
                }
                // Public Methods
                void clear(){ nPointIndex = -1; nCamIndex = -1; arrPointIndices.clear(); arrCamIndices.clear(); }
                // Public Members
                int nPointIndex;
                int nCamIndex;
                std::vector<int> arrPointIndices;
                std::vector<int> arrCamIndices;
                int nClusterIndex;
                int nLineIndex;
            };

            // Constructors and Destructors
            SFMTranscript();
            ~SFMTranscript();

            // Getters
            int numEntries() const;
            TranscriptType getTranscriptType() const;
            int numLines() const;
            std::string getLine(const int nLineIndex) const;
            std::string getEntryText(const int nIndex) const;
            EntryType getEntryType(const int nIndex) const;
            const std::vector<dlovi::Matrix> & getEntryPoints(const int nIndex) const;
            const std::vector<dlovi::Matrix> & getEntryCamCenters(const int nIndex) const;
            const std::vector<std::vector<int> > & getEntryVisList(const int nIndex) const;
            EntryType getEntryType_Step() const;
            const EntryData & getEntryData_Step() const;
            const std::vector<dlovi::Matrix> & getEntryPoints_Step() const;
            const std::vector<dlovi::Matrix> & getEntryCamCenters_Step() const;
            const std::vector<std::vector<int> > & getEntryVisList_Step() const;
            bool isIncrementalSFM() const;
            bool isValid() const;

            const std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & getEntryClusters_Step() const;
            const std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & getEntryLines_Step() const;
            const std::vector<std::vector<int> > & getEntryClusterLineList_Step() const;
            const std::vector<int> & getEntryLineVisList_Step() const;

            // Setters

            // Public Methods
            void readFromFile(const std::string & strFileName);
            void writeToFile(const std::string & strFileName) const;
            void processTranscriptText();
            void stepTranscriptText(bool bFirstEntry = false);
            void addLine(const std::string & line);
            void invalidate();

        private:
            // Private Methods
            void prepareForNewEntry();
            void prepareForNewEntry(EntryType enumEntryType);
            void prepareForNewEntry_Step(EntryType & enumEntryType, EntryType enumNewEntryType);
            int parseTranscriptHeader(int nLoop = 0);
            int parseTranscriptBody(int nLoop);
            int stepTranscriptBody(EntryType & enumEntryType, EntryData & objEntryData, std::vector<dlovi::Matrix> & arrPoints,
                                   std::vector<dlovi::Matrix> & arrCamCenters, std::vector<std::vector<int> > & arrVisLists, int nLoop);

            int stepTranscriptBody(EntryType & enumEntryType, EntryData & objEntryData, std::vector<dlovi::Matrix> & arrPoints,
                                   std::vector<dlovi::Matrix> & arrCamCenters, std::vector<std::vector<int> > & arrVisLists, int nLoop,
                                   std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & arrClusters,
                                   std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > & arrLines,
                                   std::vector<std::vector<int> > & arrClusterLineList,
                                   std::vector<int> & arrLineVisLists);

            void markAsValid();

            // Private Setters
            void setTranscriptType(TranscriptType enumTranscriptType);

            // Members
            TranscriptType m_enumTranscriptType;
            bool m_bValid;
            std::vector<std::string> m_arrStrLine; // indexed by line #

            int m_nNumEntries;
            std::vector<std::string> m_arrEntryText; // indexed by entry
            std::vector<EntryType> m_arrEntryType;// indexed by entry
            std::vector<std::vector<dlovi::Matrix> > m_arrPoints; // indexed by entry, then point index
            std::vector<std::vector<dlovi::Matrix> > m_arrCamCenters; // indexed by entry, then cam index
            std::vector<std::vector<std::vector<int> > > m_arrVisLists; // indexed by entry, then cam index.  holds point indices.

            EntryType m_enumStepEntryType;
            EntryData m_objStepEntryData;
            std::vector<dlovi::Matrix> m_arrStepPoints; // indexed by point index
            std::vector<dlovi::Matrix> m_arrStepCamCenters; // indexed by cam index
            std::vector<std::vector<int> > m_arrStepVisLists; // indexed cam index.  holds point indices.
            std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > m_arrStepClusters; // indexed by cluster index, <start,end>
            std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > m_arrStepLines; // indexed by line index, <start,end>
            std::vector<std::vector<int> > m_arrStepClusterLineList; // indexed by cluster index, holds line index
            std::vector<int> m_arrStepLineVisLists; // indexed by cam index, holds line index

        };
    }
}

#endif
