#ifndef __SFMTRANSCRIPTINTERFACE_DELAUNAY_CPP
#define __SFMTRANSCRIPTINTERFACE_DELAUNAY_CPP

#include "CARV/SFMTranscriptInterface_Delaunay.h"
#include <set>
#include "CARV/Exception.h"
#include "CARV/Matrix.h"
#include <sys/time.h>

//#include "FreespaceDelaunayAlgorithm.h"

#ifndef NULL
#define NULL 0
#endif

using namespace std;
using namespace dlovi;

// Constructors and Destructors

SFMTranscriptInterface_Delaunay::SFMTranscriptInterface_Delaunay(){
    try{
        setTranscriptRef(NULL);
        setAlgorithmRef(NULL);
        m_nCurrentEntryIndex = 0;
        m_bComputeModel = true;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "SFMTranscriptInterface_Delaunay"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

SFMTranscriptInterface_Delaunay::SFMTranscriptInterface_Delaunay(dlovi::compvis::SFMTranscript * pTranscript, dlovi::FreespaceDelaunayAlgorithm * pAlgorithm){
    try{
        setTranscriptRef(pTranscript);
        setAlgorithmRef(pAlgorithm);
        m_nCurrentEntryIndex = 0;
        m_bComputeModel = true;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "SFMTranscriptInterface_Delaunay"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

SFMTranscriptInterface_Delaunay::~SFMTranscriptInterface_Delaunay(){
    // NOP
}

// Getters

std::pair<vector<Matrix>, list<Matrix> > SFMTranscriptInterface_Delaunay::getCurrentModel() const{
    try{
        return std::make_pair(m_arrModelPoints, m_lstModelTris);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentModel"); cerr << ex2.what() << endl; //ex2.raise();
        return std::pair<vector<Matrix>, list<Matrix> >();
    }
}

dlovi::compvis::SFMTranscript::EntryType SFMTranscriptInterface_Delaunay::getCurrentEntryType() const{
    try{
        return m_pTranscript->getEntryType_Step();
        //return m_pTranscript->getEntryType(m_nCurrentEntryIndex);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryType"); cerr << ex2.what() << endl; //ex2.raise();
        return dlovi::compvis::SFMTranscript::EntryType();
    }
}

const dlovi::compvis::SFMTranscript::EntryData & SFMTranscriptInterface_Delaunay::getCurrentEntryData() const{
    try{
        return m_pTranscript->getEntryData_Step();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryData"); cerr << ex2.what() << endl; //ex2.raise();
        return *(new dlovi::compvis::SFMTranscript::EntryData); // Warning suppression; never get's executed.
    }
}

std::string SFMTranscriptInterface_Delaunay::getCurrentEntryText() const{
    try{
        throw dlovi::Exception("No entry-text available in the current implementation, TODO.");
        //return m_pTranscript->getEntryText(m_nCurrentEntryIndex);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryText"); cerr << ex2.what() << endl; //ex2.raise();
        return "";
    }
}

std::vector<dlovi::Matrix> SFMTranscriptInterface_Delaunay::getCurrentEntryPoints() const{
    try{
        return m_pTranscript->getEntryPoints_Step();
        //return m_pTranscript->getEntryPoints(m_nCurrentEntryIndex);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryPoints"); cerr << ex2.what() << endl; //ex2.raise();
        return std::vector<dlovi::Matrix>();
    }
}

std::vector<dlovi::Matrix> SFMTranscriptInterface_Delaunay::getCurrentEntryCamCenters() const{
    try{
        return m_pTranscript->getEntryCamCenters_Step();
        //return m_pTranscript->getEntryCamCenters(m_nCurrentEntryIndex);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryCamCenters"); cerr << ex2.what() << endl; //ex2.raise();
        return std::vector<dlovi::Matrix>();
    }
}

std::vector<std::vector<int> > SFMTranscriptInterface_Delaunay::getCurrentEntryVisList() const{
    try{
        return m_pTranscript->getEntryVisList_Step();
        //return m_pTranscript->getEntryVisList(m_nCurrentEntryIndex);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryVisList"); cerr << ex2.what() << endl; //ex2.raise();
        return std::vector<std::vector<int> >();
    }
}

int SFMTranscriptInterface_Delaunay::numFreeSpaceConstraintsInTriangulation() const{
    try{
        // Compute and return the # of freespace constraints in the triangulation for timing/complexity analysis information
        std::set<pair<int, int>, dlovi::FreespaceDelaunayAlgorithm::Delaunay3CellInfo::LtConstraint> setAllConstraints;
        for(dlovi::FreespaceDelaunayAlgorithm::Delaunay3::Finite_cells_iterator itAllCells = m_objDelaunay.finite_cells_begin();
            itAllCells != m_objDelaunay.finite_cells_end(); itAllCells++)
            setAllConstraints.insert(itAllCells->info().getIntersections().begin(), itAllCells->info().getIntersections().end());
        return (int) setAllConstraints.size();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "numFreeSpaceConstraintsInTriangulation"); cerr << ex2.what() << endl; //ex2.raise();
        return -1;
    }
}

std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > SFMTranscriptInterface_Delaunay::getCurrentEntryClusters() const{
    try{
        return m_pTranscript->getEntryClusters_Step();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryClusters"); cerr << ex2.what() << endl; //ex2.raise();
        return std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> >();
    }
}

std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> > SFMTranscriptInterface_Delaunay::getCurrentEntryLines() const{
    try{
        return m_pTranscript->getEntryLines_Step();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryLines"); cerr << ex2.what() << endl; //ex2.raise();
        return std::vector<std::pair<dlovi::Matrix,dlovi::Matrix> >();
    }
}

std::vector<std::vector<int> > SFMTranscriptInterface_Delaunay::getCurrentEntryClusterLineList() const{
    try{
        return m_pTranscript->getEntryClusterLineList_Step();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryClusterLineList"); cerr << ex2.what() << endl; //ex2.raise();
        return std::vector<std::vector<int> >();
    }
}
std::vector<int> SFMTranscriptInterface_Delaunay::getCurrentEntryLineVisList() const{
    try{
        return m_pTranscript->getEntryLineVisList_Step();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "getCurrentEntryLineVisList"); cerr << ex2.what() << endl; //ex2.raise();
        return std::vector<int>();
    }
}

bool SFMTranscriptInterface_Delaunay::isComputingModel() const{
    return m_bComputeModel;
}

// Setters

void SFMTranscriptInterface_Delaunay::setTranscriptRef(dlovi::compvis::SFMTranscript * pTranscript){
    try{
        m_pTranscript = pTranscript;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "setTranscriptRef"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::setAlgorithmRef(dlovi::FreespaceDelaunayAlgorithm * pAlgorithm){
    try{
        m_pAlgorithm = pAlgorithm;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "setAlgorithmRef"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::setComputingModel(bool compute){
    m_bComputeModel = compute;
}

// Public Methods

void SFMTranscriptInterface_Delaunay::loadTranscriptFromFile(const std::string & strFileName){
    try{
        m_pTranscript->readFromFile(strFileName);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "loadTranscriptFromFile"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::processTranscript(){
    try{
        // WARNING: Takes a LOT of memory, and also NOT very useful since this interface doesn't expose the processing results.
        m_pTranscript->processTranscriptText();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "processTranscript"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::stepTranscript(bool bFirstEntry){
    try{
        m_pTranscript->stepTranscriptText(bFirstEntry);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "stepTranscript"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::runFull(){
    try{
        rewind();
        while(!isDone())
            step();

        // TODO: Debug: Remove Me.
        cerr << "numGiantPoints: " << m_setGiantPoints.size() << endl;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "runFull"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::runOnlyFinalState(){
    try{
        // Go to the end of the transcript
        rewind();
        while(!isDone())
            step(false);

        // Feed the final map, cameras, and visibility information to the algorithm
        m_pAlgorithm->setCamCenters(getCurrentEntryCamCenters());
        m_pAlgorithm->setPoints(getCurrentEntryPoints());
        m_pAlgorithm->setVisibilityList(getCurrentEntryVisList());

        // Calculate the bounding vertices values
        m_pAlgorithm->calculateBoundsValues();

        // Run the algorithm incrementally, one keyframe at a time
        for(int i = 0; i < m_pAlgorithm->numCams(); i++)
            m_pAlgorithm->IterateTetrahedronMethod(m_objDelaunay, m_arrVertexHandles, i);

        // Finally, compute the isosurface model
        computeCurrentModel();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "runOnlyFinalState"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::runRemainder(){
    try{
        while(!isDone())
            step();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "runRemainder"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::step(bool bRunAlgorithm){
    try{
        stepTranscript(m_nCurrentEntryIndex == 0);

        if(bRunAlgorithm){
            if(getCurrentEntryType() == dlovi::compvis::SFMTranscript::ET_RESET){
                m_arrVertexHandles.clear();
                m_objDelaunay.clear();
                *m_pAlgorithm = dlovi::FreespaceDelaunayAlgorithm();
                m_lstModelTris.clear();
                m_arrModelPoints.clear();
                m_setGiantPoints.clear();
            }
            else if(getCurrentEntryType() == dlovi::compvis::SFMTranscript::ET_POINTDELETION){
                // m_pAlgorithm->setPoints(getCurrentEntryPoints()); // The point is left as a ghost entry, so no modification to the points array takes place.
                m_pAlgorithm->setVisibilityList(getCurrentEntryVisList()); // The visibility list, however, changes to remove refs to this point.
                int nPointIndex = getCurrentEntryData().nPointIndex;

                if(m_setGiantPoints.count(nPointIndex) == 0){
                    m_pAlgorithm->removeVertex(m_objDelaunay, m_arrVertexHandles, nPointIndex);
                    computeCurrentModel();
                }
            }
            else if(getCurrentEntryType() == dlovi::compvis::SFMTranscript::ET_VISIBILITYRAYINSERTION){
                m_pAlgorithm->setVisibilityList(getCurrentEntryVisList());
                int nCamIndex = getCurrentEntryData().nCamIndex;
                int nPointIndex = getCurrentEntryData().nPointIndex;

                if(m_setGiantPoints.count(nPointIndex) == 0){
                    m_pAlgorithm->applyConstraint(m_objDelaunay, m_arrVertexHandles, nCamIndex, nPointIndex);
                    computeCurrentModel();
                }
            }
            else if(getCurrentEntryType() == dlovi::compvis::SFMTranscript::ET_VISIBILITYRAYDELETION){
                m_pAlgorithm->setVisibilityList(getCurrentEntryVisList());
                int nCamIndex = getCurrentEntryData().nCamIndex;
                int nPointIndex = getCurrentEntryData().nPointIndex;

                if(m_setGiantPoints.count(nPointIndex) == 0){
                    m_pAlgorithm->removeConstraint(m_objDelaunay, m_arrVertexHandles, nCamIndex, nPointIndex);
                    computeCurrentModel();
                }
            }
            else if(getCurrentEntryType() == dlovi::compvis::SFMTranscript::ET_KEYFRAMEINSERTION){
                // TODO: modify the point addition algorithm to carve away the new points' FULL vis lists,
                // not just the current KF. (it's missing the epipolar match visibility ray)
                int nStartPointIndex = m_pAlgorithm->numPoints();

                m_pAlgorithm->setCamCenters(getCurrentEntryCamCenters());
                m_pAlgorithm->setPoints(getCurrentEntryPoints());
                int nCamIndex = getCurrentEntryData().nCamIndex;

                int nEndPointIndex = m_pAlgorithm->numPoints() - 1;

                // Test if any of the new points are too large, and if so handle them specially
                for(int nLoop = nStartPointIndex; nLoop <= nEndPointIndex; nLoop++){
                    if(isPointTooLarge(m_pAlgorithm->getPoint(nLoop)))
                        m_setGiantPoints.insert(nLoop);
                }

                // Filter giant points out of the current view's visibility list, so that the KF-addition alg. doesn't add them
                std::vector<std::vector<int> > arrTmpVisList = getCurrentEntryVisList();
                m_pAlgorithm->setVisibilityList(filterOutGiantPointsFromCurrentVisList(arrTmpVisList));

                m_pAlgorithm->IterateTetrahedronMethod(m_objDelaunay, m_arrVertexHandles, nCamIndex);
                computeCurrentModel();
            }
            else if(getCurrentEntryType() == dlovi::compvis::SFMTranscript::ET_BUNDLEADJUSTMENT){
                m_pAlgorithm->setCamCenters(getCurrentEntryCamCenters());
                m_pAlgorithm->setPoints(getCurrentEntryPoints());

                // Test if any of the moved points become (or are) too large, and if so handle them specially
                std::vector<int> arrTmpFilteredPointIndices = getCurrentEntryData().arrPointIndices;
                filterOutGiantPoints(arrTmpFilteredPointIndices);
                std::vector<int> arrNewlyGiantPointIndices;
                std::vector<int> arrFilteredPointIndices;
                for(int nLoop = 0; nLoop < (int)arrTmpFilteredPointIndices.size(); nLoop++){
                    if(isPointTooLarge(m_pAlgorithm->getPoint(arrTmpFilteredPointIndices[nLoop]))){
                        m_setGiantPoints.insert(arrTmpFilteredPointIndices[nLoop]);
                        arrNewlyGiantPointIndices.push_back(arrTmpFilteredPointIndices[nLoop]);
                    }
                    else
                        arrFilteredPointIndices.push_back(arrTmpFilteredPointIndices[nLoop]);
                }

                // Remove any newly giant points from the triangulation (points bundled to giant locations)
                // TODO: implement a batch-removal algorithm (the current implementation is buggy).
                for(std::vector<int>::iterator itDel = arrNewlyGiantPointIndices.begin(); itDel != arrNewlyGiantPointIndices.end(); itDel++)
                    m_pAlgorithm->removeVertex(m_objDelaunay, m_arrVertexHandles, *itDel);

                // Perform the bundle adjustment on the triangulation
                // TODO: implement cam-center-move algorithm for bundle adjustments & call here.
                // For now, and perhaps good enough in practice, it just does point moves.
                m_pAlgorithm->moveVertex(m_objDelaunay, m_arrVertexHandles, arrFilteredPointIndices);

                // Old Code handles each point in sequence.  Too Slow:
                //for(std::vector<int>::const_iterator itPointIndex = getCurrentEntryData().arrPointIndices.begin();
                //itPointIndex != getCurrentEntryData().arrPointIndices.end(); itPointIndex++)
                //  m_pAlgorithm->moveVertex(m_objDelaunay, m_arrVertexHandles, *itPointIndex);

                computeCurrentModel();
            }
            else if(getCurrentEntryType() == dlovi::compvis::SFMTranscript::ET_INVALID){
                throw dlovi::Exception("Invalid log entry.");
            }
            else{
                // TODO: Ignore this entry for now (only PTAM subset supported).  Perhaps implement more functionality later.
            }
        }

        m_nCurrentEntryIndex++;
    }
    catch(std::exception & ex){
        m_nCurrentEntryIndex++;
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "step"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_Delaunay::rewind(){
    try{
        m_arrVertexHandles.clear();
        m_objDelaunay.clear();
        *m_pAlgorithm = dlovi::FreespaceDelaunayAlgorithm();
        m_lstModelTris.clear();
        m_arrModelPoints.clear();
        m_setGiantPoints.clear();
        m_nCurrentEntryIndex = 0;
        m_pTranscript->invalidate();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "rewind"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

bool SFMTranscriptInterface_Delaunay::isDone(){
    try{
        return m_pTranscript->isValid();
        //return m_nCurrentEntryIndex == m_pTranscript->numEntries();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "isDone"); cerr << ex2.what() << endl; //ex2.raise();
        return false;
    }
}

void SFMTranscriptInterface_Delaunay::writeCurrentModelToFile(const std::string & strFileName) const{
    try{
        m_pAlgorithm->writeObj(strFileName, m_arrModelPoints, m_lstModelTris);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "writeCurrentModelToFile"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

// Private Methods

void SFMTranscriptInterface_Delaunay::computeCurrentModel(int nVoteThresh){
    try{
        // Only recomputes the model if enough time has passed.  Otherwise model updates are too frequent and bog the cpu.
        static double then = timestamp();
        double now = timestamp();
        double dt = now - then;

        if(m_bComputeModel && dt > 5.0){ // more than 5 seconds passed

            m_pAlgorithm->tetsToTris(m_objDelaunay, m_arrModelPoints, m_lstModelTris, nVoteThresh);
            then = now;
            std::cout << "model points: " << m_arrModelPoints.size() << " tris:" << m_lstModelTris.size() << std::endl;

        }
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "computeCurrentModel"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

bool SFMTranscriptInterface_Delaunay::isPointTooLarge(const dlovi::Matrix & matPoint) const{
    try{
        double dMin = m_pAlgorithm->getBoundsMin() + dlovi::eps_d;
        double dMax = m_pAlgorithm->getBoundsMax() - dlovi::eps_d;
        return (matPoint(0) > dMax) || (matPoint(1) > dMax) || (matPoint(2) > dMax) || (matPoint(0) < dMin) || (matPoint(1) < dMin) || (matPoint(2) < dMin);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "isPointTooLarge"); cerr << ex2.what() << endl; //ex2.raise()
        return false;
    }
}

std::vector<int> & SFMTranscriptInterface_Delaunay::filterOutGiantPoints(std::vector<int> & arrPointIndices) const{
    try{
        std::vector<int> arrNewIndices;

        for(int nLoop = 0; nLoop < (int)arrPointIndices.size(); nLoop++){
            if(m_setGiantPoints.count(arrPointIndices[nLoop]) == 0)
                arrNewIndices.push_back(arrPointIndices[nLoop]);
        }

        arrPointIndices = arrNewIndices;

        return arrPointIndices;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "filterOutGiantPoints"); cerr << ex2.what() << endl; //ex2.raise()
        return arrPointIndices;
    }
}

std::vector<std::vector<int> > & SFMTranscriptInterface_Delaunay::filterOutGiantPointsFromCurrentVisList(std::vector<std::vector<int> > & arrVisLists) const{
    try{
        filterOutGiantPoints(arrVisLists[(int)arrVisLists.size() - 1]);
        return arrVisLists;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "filterOutGiantPointsFromCurrentVisList"); cerr << ex2.what() << endl; //ex2.raise()
        return arrVisLists;
    }
}

double SFMTranscriptInterface_Delaunay::timestamp() const{
    timeval t;
    gettimeofday(&t, 0);
    return (double)(t.tv_sec + (t.tv_usec / 1000000.0));
}


#endif
