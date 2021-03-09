#ifndef __SFMTRANSCRIPTINTERFACE_DELAUNAY_H
#define __SFMTRANSCRIPTINTERFACE_DELAUNAY_H

#include "CARV/SFMTranscript.h"
#include "CARV/FreespaceDelaunayAlgorithm.h"
#include <vector>
#include <string>
#include <utility>

class SFMTranscriptInterface_Delaunay;

namespace dlovi{
    class FreespaceDelaunayAlgorithm;
}

class SFMTranscriptInterface_Delaunay{
public:
    // Constructors and Destructors
    SFMTranscriptInterface_Delaunay();
    SFMTranscriptInterface_Delaunay(dlovi::compvis::SFMTranscript * pTranscript, dlovi::FreespaceDelaunayAlgorithm * pAlgorithm);
    ~SFMTranscriptInterface_Delaunay();

    // Getters
    std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > getCurrentModel() const;
    dlovi::compvis::SFMTranscript::EntryType getCurrentEntryType() const;
    const dlovi::compvis::SFMTranscript::EntryData & getCurrentEntryData() const;
    std::string getCurrentEntryText() const;
    std::vector<dlovi::Matrix> getCurrentEntryPoints() const;
    std::vector<dlovi::Matrix> getCurrentEntryCamCenters() const;
    std::vector<std::vector<int> > getCurrentEntryVisList() const;
    int numFreeSpaceConstraintsInTriangulation() const;

    std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > getCurrentEntryClusters() const;
    std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > getCurrentEntryLines() const;
    std::vector<std::vector<int> > getCurrentEntryClusterLineList() const;
    std::vector<int> getCurrentEntryLineVisList() const;
    bool isComputingModel() const;

    // Setters
    void setTranscriptRef(dlovi::compvis::SFMTranscript * pTranscript);
    void setAlgorithmRef(dlovi::FreespaceDelaunayAlgorithm * pAlgorithm);
    void setComputingModel(bool compute);

    // Public Methods
    void loadTranscriptFromFile(const std::string & strFileName);
    void processTranscript();
    void stepTranscript(bool bFirstEntry = false);
    void runFull();
    void runOnlyFinalState();
    void runRemainder();
    void step(bool bRunAlgorithm = true);
    void rewind();
    bool isDone();
    void writeCurrentModelToFile(const std::string & strFileName) const;

    void computeCurrentModel(int nVoteThresh = 1);

private:
    // Private Methods
    bool isPointTooLarge(const dlovi::Matrix & matPoint) const;
    std::vector<int> & filterOutGiantPoints(std::vector<int> & arrPointIndices) const;
    std::vector<std::vector<int> > & filterOutGiantPointsFromCurrentVisList(std::vector<std::vector<int> > & arrVisLists) const;
    double timestamp() const;

    // Member Variables
    dlovi::compvis::SFMTranscript * m_pTranscript;
    dlovi::FreespaceDelaunayAlgorithm * m_pAlgorithm;
    dlovi::FreespaceDelaunayAlgorithm::Delaunay3 m_objDelaunay;
    std::vector<dlovi::FreespaceDelaunayAlgorithm::Delaunay3::Vertex_handle> m_arrVertexHandles;
    std::vector<dlovi::Matrix> m_arrModelPoints;
    std::list<dlovi::Matrix> m_lstModelTris;
    int m_nCurrentEntryIndex;
    std::set<int> m_setGiantPoints;
    bool m_bComputeModel; // used to stop computing the model
};

#endif

