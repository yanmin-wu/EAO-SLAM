//  Copyright 2011 David Lovi
//
//  This file is part of FreespaceDelaunayAlgorithm.
//
//  FreespaceDelaunayAlgorithm is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version.
//
//  FreespaceDelaunayAlgorithm is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with FreespaceDelaunayAlgorithm.  If not, see
//  <http://www.gnu.org/licenses/>.
//
//  As a special exception, you have permission to link this program
//  with the CGAL library and distribute executables, as long as you
//  follow the requirements of the GNU GPL in regard to all of the
//  software in the executable aside from CGAL.


#ifndef __FREESPACEDELAUNAYALGORITHM_CPP
#define __FREESPACEDELAUNAYALGORITHM_CPP

#include "CARV/FreespaceDelaunayAlgorithm.h"
#include <sys/time.h>
#include <algorithm>

namespace dlovi {

    // Constructors

    FreespaceDelaunayAlgorithm::FreespaceDelaunayAlgorithm() {
        calculateBoundsValues();
    }

    FreespaceDelaunayAlgorithm::FreespaceDelaunayAlgorithm(const vector<Matrix> & points, const vector<Matrix> & cams, const vector<Matrix> & camCenters,
                                                           const vector<Matrix> & principleRays, const vector<vector<int> > & visibilityList) {
        copy(points, cams, camCenters, principleRays, visibilityList);
        calculateBoundsValues();
    }

    FreespaceDelaunayAlgorithm::FreespaceDelaunayAlgorithm(const vector<Matrix> & points, const vector<Matrix> & cams, const vector<Matrix> & camCenters,
                                                           const vector<Matrix> & principleRays, const vector<Matrix> & normals) {
        vector<vector<int> > visibilityList;
        copy(points, cams, camCenters, principleRays, visibilityList);
        calculateBoundsValues();

        // Construct visibility list
        generateVisibilityFromNormals(normals);
    }

    FreespaceDelaunayAlgorithm::FreespaceDelaunayAlgorithm(const FreespaceDelaunayAlgorithm & ref) {
        copy(ref.getPoints(), ref.getCams(), ref.getCamCenters(), ref.getPrincipleRays(), ref.getVisibilityList());
        m_mapPoint_VertexHandle = ref.m_mapPoint_VertexHandle;
        calculateBoundsValues();
    }

    // Getters

    const vector<Matrix> & FreespaceDelaunayAlgorithm::getPoints() const {
        return m_points;
    }

    Matrix FreespaceDelaunayAlgorithm::getPoint(const int index) const {
        return m_points[index];
    }

    int FreespaceDelaunayAlgorithm::numPoints() const {
        return (int)m_points.size();
    }

    const vector<Matrix> & FreespaceDelaunayAlgorithm::getCams() const {
        return m_cams;
    }

    Matrix FreespaceDelaunayAlgorithm::getCam(const int index) const {
        return m_cams[index];
    }

    const vector<Matrix> & FreespaceDelaunayAlgorithm::getCamCenters() const {
        return m_camCenters;
    }

    Matrix FreespaceDelaunayAlgorithm::getCamCenter(const int index) const {
        return m_camCenters[index];
    }

    const vector<Matrix> & FreespaceDelaunayAlgorithm::getPrincipleRays() const {
        return m_principleRays;
    }

    Matrix FreespaceDelaunayAlgorithm::getPrincipleRay(const int index) const {
        return m_principleRays[index];
    }

    const vector<vector<int> > & FreespaceDelaunayAlgorithm::getVisibilityList() const {
        return m_visibilityList;
    }

    const vector<int> & FreespaceDelaunayAlgorithm::getVisibilityList(const int index) const {
        return m_visibilityList[index];
    }

    int FreespaceDelaunayAlgorithm::numCams() const {
        return (int)m_camCenters.size();
    }

    double FreespaceDelaunayAlgorithm::getBoundsMin() const {
        return m_nBoundsMin;
    }

    double FreespaceDelaunayAlgorithm::getBoundsMax() const {
        return m_nBoundsMax;
    }

    // Setters

    void FreespaceDelaunayAlgorithm::setPoints(const vector<Matrix> & ref) {
        m_points = ref;
    }

    void FreespaceDelaunayAlgorithm::setCams(const vector<Matrix> & ref) {
        m_cams = ref;
    }

    void FreespaceDelaunayAlgorithm::setCamCenters(const vector<Matrix> & ref) {
        m_camCenters = ref;
    }

    void FreespaceDelaunayAlgorithm::setPrincipleRays(const vector<Matrix> & ref) {
        m_principleRays = ref;
    }

    void FreespaceDelaunayAlgorithm::setVisibilityList(const vector<vector<int> > & ref) {
        m_visibilityList = ref;
    }

    void FreespaceDelaunayAlgorithm::addPoint(const Matrix & ref) {
        m_points.push_back(ref);
    }

    void FreespaceDelaunayAlgorithm::addCamCenter(const Matrix & ref) {
        m_camCenters.push_back(ref);
        m_visibilityList.push_back(vector<int>());
    }

    void FreespaceDelaunayAlgorithm::addVisibilityPair(const int camIndex, const int pointIndex) {
        m_visibilityList[camIndex].push_back(pointIndex);
    }

    void FreespaceDelaunayAlgorithm::addVisibilityPair(const std::pair<int, int> visibilityPair) {
        m_visibilityList[visibilityPair.first].push_back(visibilityPair.second);
    }

    // Operators

    FreespaceDelaunayAlgorithm & FreespaceDelaunayAlgorithm::operator=(const FreespaceDelaunayAlgorithm & rhs) {
        if (this != & rhs) {
            copy(rhs.getPoints(), rhs.getCams(), rhs.getCamCenters(), rhs.getPrincipleRays(), rhs.getVisibilityList());
            m_mapPoint_VertexHandle = rhs.m_mapPoint_VertexHandle;
            calculateBoundsValues();
        }
        return *this;
    }

    // Public Methods

    bool FreespaceDelaunayAlgorithm::isVisible(const int pointIndex, const int viewIndex) const {
        const vector<int> & visList = getVisibilityList(viewIndex);

        for (vector<int>::const_iterator it = visList.begin(); it != visList.end(); it++) {
            if (*it == pointIndex)
                return true;
        }
        return false;
    }

    void FreespaceDelaunayAlgorithm::generateVisibilityFromNormals(const vector<Matrix> & normals, const double nFrontFacingAngleThreshold, const double nFov) {
        vector<vector<int> > tmpVisibility(numCams());
        Matrix vCamToPoint;

        for (int i = 0; i < numCams(); i++) {
            for (int j = 0; j < numPoints(); j++) {
                vCamToPoint = getPoint(j) - getCamCenter(i);
                if (normals[j].angleBetween(-vCamToPoint) < nFrontFacingAngleThreshold && vCamToPoint.angleBetween(getPrincipleRay(i)) < nFov / 2.0)
                    tmpVisibility[i].push_back(j);
            }
        }
        setVisibilityList(tmpVisibility);
    }

    void FreespaceDelaunayAlgorithm::TetrahedronBatchMethod(Delaunay3 & dt, const bool bUseViewingOrder) const {
        // Initialize the model as empty
        if (dt.number_of_vertices() > 0) dt.clear();

        // Create the bounding vertices as the first 8 vertices in dt: (a loose bounding box).
        createBounds(dt);

        vector<Delaunay3::Vertex_handle> vecVertexHandles;
        vector<int> localVisLists[numCams()];

        // Add all the observed vertices to the triangulation.  On construction, the voting counts of all cells are zero.
        // Also build the list of cached vertex handles into the triangulation.
        if (! bUseViewingOrder) {
            for (int i = 0; i < numPoints(); i++) {
                vecVertexHandles.push_back(dt.insert(PointD3(getPoint(i)(0), getPoint(i)(1), getPoint(i)(2))));
                m_mapPoint_VertexHandle[i] = i;
            }
        }
        else {
            for (int frameIndex = 0; frameIndex < numCams(); frameIndex++) {
                // Add new features to the global model that weren't observed before, and construct the local visibility list:
                vector<int>::const_iterator itOriginalLocalVisList;
                for (itOriginalLocalVisList = getVisibilityList(frameIndex).begin(); itOriginalLocalVisList != getVisibilityList(frameIndex).end(); itOriginalLocalVisList++) {
                    int originalIndex = *itOriginalLocalVisList;
                    Matrix matTmpPoint = getPoint(originalIndex);
                    PointD3 pd3TmpPoint(matTmpPoint(0), matTmpPoint(1), matTmpPoint(2));

                    map<int, int>::iterator itVertHandleIndex = m_mapPoint_VertexHandle.find(originalIndex);
                    if (itVertHandleIndex != m_mapPoint_VertexHandle.end()) {
                        // We found a match: this feature is already in the point set.  So add it to the visibility list.
                        localVisLists[frameIndex].push_back(itVertHandleIndex->second);
                    }
                    else {
                        // We didn't find a match, so this is a new feature.  Add it to the point set and the visibility list.
                        vecVertexHandles.push_back(dt.insert(pd3TmpPoint));
                        m_mapPoint_VertexHandle[originalIndex] = (int)vecVertexHandles.size() - 1;
                        localVisLists[frameIndex].push_back((int)vecVertexHandles.size() - 1);
                    }
                }
            }
        }

        // Iterate over views; iterate over visible points:
        if (! bUseViewingOrder) {
            for (int i = 0; i < numCams(); i++) {
                const vector<int> & localVisList = getVisibilityList(i);
                for (int j = 0; j < (int)localVisList.size(); j++) {
                    // let Q be the point & O the optic center.
                    Matrix matQ = getPoint(localVisList[j]);
                    Matrix matO = getCamCenter(i);
                    PointD3 Q(matQ(0), matQ(1), matQ(2));
                    PointD3 O(matO(0), matO(1), matO(2));
                    Segment QO = Segment(Q, O);
                    Delaunay3::Vertex_handle hndlQ = vecVertexHandles[localVisList[j]];

                    // Increment the voting counts of all tetrahedra that intersect the constraint QO
                    markTetrahedraCrossingConstraint(dt, hndlQ, QO);
                }
            }
        }
        else {
            for (int i = 0; i < numCams(); i++) {
                for (int j = 0; j < (int)localVisLists[i].size(); j++) {
                    // let Q be the point & O the optic center.
                    Matrix matO = getCamCenter(i);
                    PointD3 Q(vecVertexHandles[localVisLists[i][j]]->point());
                    PointD3 O(matO(0), matO(1), matO(2));
                    Segment QO = Segment(Q, O);
                    Delaunay3::Vertex_handle hndlQ = vecVertexHandles[localVisLists[i][j]];

                    // Increment the voting counts of all tetrahedra that intersect the constraint QO
                    markTetrahedraCrossingConstraint(dt, hndlQ, QO);
                }
            }
        }
        // Done marking tetrahedra; return.
    }

    void FreespaceDelaunayAlgorithm::IterateTetrahedronMethod(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int frameIndex) const {
        // If the bounding vertices doesn't exist, create it as the first 8 vertices in dt:
        if (dt.number_of_vertices() == 0)
            createBounds(dt);

        // Add the newly observed points into the triangulation and the vertex handle list.
        // This involves, for each new point, properly deleting & staring off a connected subset of tetrahedra that violate the Delaunay constraint,
        // while marking the new tetrahedra with the deleted tetrahedra's freespace constraints.
        vector<int> localVisList;
        addNewlyObservedFeatures(dt, vecVertexHandles, localVisList, getVisibilityList(frameIndex));

        // Apply the current view's freespace constraints to the triangulation
        Matrix matO = getCamCenter(frameIndex);
        PointD3 O(matO(0), matO(1), matO(2));
        for (int j = 0; j < (int)localVisList.size(); j++) {
            // let Q be the point & O the optic center.
            Delaunay3::Vertex_handle hndlQ = vecVertexHandles[localVisList[j]];

            // TODO: DEBUG: PTAM has minor data corruption bugs, and this hack handles the bad data being passed to our code.  Should fix PTAM instead.
            if (! dt.is_vertex(hndlQ))
                continue;

            Segment QO = Segment(hndlQ->point(), O);

            // Increment the voting counts of all tetrahedra that intersect the constraint QO & keep track of which constraints crossed which tetrahedra.
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, hndlQ, QO, frameIndex, localVisList[j]);
        }
        // Done marking tetrahedra; return.
    }


    void FreespaceDelaunayAlgorithm::removeVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int pointIndex) const {
        // Vertex Deletion Algorithm:
        // ~~~~~~~~~~~~~~~~~~~~
        // Step 1: Collect FS constraints into a unioned set from incident cells.  Don't add FS constraints containing the vertex to be deleted.
        // Step 2: Delete the vertex (this retriangulates).
        // Step 3: Iterate over all cells and remove any FS constraints containing the deleted vertex.  Meanwhile determine the set of new cells.
        // Step 4: Process the FS Constraints in the unioned set.  Mark new cells as old.

        Delaunay3::Vertex_handle hndlQ;
        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setUnionedConstraints;
        set<Delaunay3::Cell_handle> setNewCells;

        int vertexIndex = m_mapPoint_VertexHandle[pointIndex];

        hndlQ = vecVertexHandles[vertexIndex];

        // TODO：DEBUG data corruption
        if (! dt.is_vertex(hndlQ))
            return;

        // Step 1:
        set<Delaunay3::Cell_handle> setIncidentCells;
        dt.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

        for (set<Delaunay3::Cell_handle>::iterator itCell = setIncidentCells.begin(); itCell != setIncidentCells.end(); itCell++) {
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itConstraint = (*itCell)->info().getIntersections().begin();
                 itConstraint != (*itCell)->info().getIntersections().end(); itConstraint++) {
                if (itConstraint->second != vertexIndex)
                    setUnionedConstraints.insert(*itConstraint);
            }
        }

        // Step 2:
        dt.remove(hndlQ);
        vecVertexHandles[vertexIndex] = Delaunay3::Vertex_handle();

        // Step 3:
        for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin(); itCell != dt.finite_cells_end(); itCell++) {
            if (itCell->info().isNew())
                setNewCells.insert(itCell);
            // Linear search:
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itDelete = itCell->info().getIntersections().begin();
                 itDelete != itCell->info().getIntersections().end(); ) {
                if (itDelete->second == vertexIndex) {
                    // invalidates iterator, so careful about incrementing it:
                    set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itNext = itDelete;
                    itNext++;
                    itCell->info().removeIntersection(itDelete->first, itDelete->second, vecVertexHandles, getCamCenters());
                    itCell->info().decrementVoteCount();
                    itDelete = itNext;
                }
                else
                    itDelete++;
            }
        }

        // Step 4:
        for (set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedConstraints.begin(); itConstraint != setUnionedConstraints.end(); itConstraint++) {
            Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
                                 PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[itConstraint->second], QO, itConstraint->first, itConstraint->second, true);
        }
        for (set<Delaunay3::Cell_handle>::iterator itCell = setNewCells.begin(); itCell != setNewCells.end(); itCell++)
            (*itCell)->info().markOld();
    }

    // TODO: The following function is both buggy and untested.  We've used the single-vertex-deletion version of this function in our testing and experiments.
    void FreespaceDelaunayAlgorithm::removeVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const set<int> & setPointIndices) const {
        // Vertex Deletion Algorithm:
        // ~~~~~~~~~~~~~~~~~~~~
        // Step 1: Collect FS constraints into a unioned set from incident cells.  Don't add FS constraints containing the vertex to be deleted.
        // Step 2: Delete the vertex (this retriangulates).
        // Step 3: Iterate steps 1 and 2 for each vertex to be deleted
        // Step 4: Iterate over all cells and remove any FS constraints containing the deleted vertices.  Meanwhile determine the set of new cells.
        // Step 5: Process the FS Constraints in the unioned set.  Mark new cells as old.

        Delaunay3::Vertex_handle hndlQ;
        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setUnionedConstraints;
        set<int> setRemovedVertexIndices;

        for (set<int>::const_iterator itPointIndex = setPointIndices.begin(); itPointIndex != setPointIndices.end(); itPointIndex++) {
            int pointIndex = *itPointIndex;
            int vertexIndex = m_mapPoint_VertexHandle[pointIndex];
            hndlQ = vecVertexHandles[vertexIndex];

            // Step 1:
            set<Delaunay3::Cell_handle> setIncidentCells;
            dt.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

            for (set<Delaunay3::Cell_handle>::iterator itCell = setIncidentCells.begin(); itCell != setIncidentCells.end(); itCell++) {
                for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itConstraint = (*itCell)->info().getIntersections().begin();
                     itConstraint != (*itCell)->info().getIntersections().end(); itConstraint++) {
                    // TODO: BUG: The following conditional doesn't test against all vertexIndices that will be removed, just the current one.  Wrong!  Should prune this set.
                    if(itConstraint->second != vertexIndex)
                        setUnionedConstraints.insert(*itConstraint);
                }
            }

            // Step 2:
            dt.remove(hndlQ);
            setRemovedVertexIndices.insert(setRemovedVertexIndices.end(), vertexIndex);
        }

        // Step 4:
        set<Delaunay3::Cell_handle> setNewCells;
        for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin(); itCell != dt.finite_cells_end(); itCell++) {
            if (itCell->info().isNew())
                setNewCells.insert(itCell);
            // Linear search:
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itDelete = itCell->info().getIntersections().begin();
                 itDelete != itCell->info().getIntersections().end(); ) {
                if (setRemovedVertexIndices.count(itDelete->second) > 0) {
                    // invalidates iterator, so careful about incrementing it:
                    set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itNext = itDelete;
                    itNext++;
                    itCell->info().removeIntersection(itDelete->first, itDelete->second, vecVertexHandles, getCamCenters());
                    itCell->info().decrementVoteCount();
                    itDelete = itNext;
                }
                else
                    itDelete++;
            }
        }

        // Step 5:
        for (set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedConstraints.begin(); itConstraint != setUnionedConstraints.end(); itConstraint++) {
            Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
                                 PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[itConstraint->second], QO, itConstraint->first, itConstraint->second, true);
        }
        for (set<Delaunay3::Cell_handle>::iterator itCell = setNewCells.begin(); itCell != setNewCells.end(); itCell++)
            (*itCell)->info().markOld();
    }

    void FreespaceDelaunayAlgorithm::moveVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int pointIndex) const {
        // Vertex Moving Algorithm:
        // ~~~~~~~~~~~~~~~~~~~~
        // Step 1: Collect FS constraints into two unioned sets from incident cells.  FS constraints containing the vertex to be moved go to their own set.
        //				(Since they will refer to a new point location)
        // Step 2: Delete the vertex (this retriangulates).
        // Step 3: As in point insertion, find the set of Delaunay-conflicting cells for the moved point, and add their FS constraints to the unioned sets.
        //				(Again same division between sets)
        // Step 4: Insert the point into the triangulation (this retriangulates).
        // Step 5: Repeat steps 1-4 for each vertex to be moved.
        // Step 6: Iterate over all cells in the DT and remove any FS constraints containing the deleted vertices.  Meanwhile determine the set of new cells.
        // Step 7: Process the FS constraints in the unioned sets.  Normal constraints only mark new cells.  Special constraints containing the moved
        //				points mark all crossed cells (as they were explicitly deleted in step 6).  Mark new cells as old.

        Delaunay3::Vertex_handle hndlQ;
        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setUnionedStationaryConstraints;
        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setUnionedMovedConstraints;
        set<Delaunay3::Cell_handle> setNewCells;

        int vertexIndex = m_mapPoint_VertexHandle[pointIndex];
        hndlQ = vecVertexHandles[vertexIndex];
        PointD3 pd3NewPoint(getPoint(pointIndex)(0), getPoint(pointIndex)(1), getPoint(pointIndex)(2));

        // TODO: DEBUG: PTAM has minor data corruption bugs, and this hack handles the bad data being passed to our code.  Should fix PTAM instead.
        if (! dt.is_vertex(hndlQ))
            return;

        // Step 1:
        set<Delaunay3::Cell_handle> setIncidentCells;
        dt.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

        for (set<Delaunay3::Cell_handle>::iterator itCell = setIncidentCells.begin(); itCell != setIncidentCells.end(); itCell++) {
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itConstraint = (*itCell)->info().getIntersections().begin();
                 itConstraint != (*itCell)->info().getIntersections().end(); itConstraint++) {
                if (itConstraint->second == vertexIndex)
                    setUnionedMovedConstraints.insert(*itConstraint);
                else
                    setUnionedStationaryConstraints.insert(*itConstraint);
            }
        }

        // Step 2:
        dt.remove(hndlQ);

        // Step 3:
        // Locate the point
        Delaunay3::Locate_type lt;
        int li, lj;
        Delaunay3::Cell_handle c = dt.locate(pd3NewPoint, lt, li, lj);
        if (lt == Delaunay3::VERTEX) {
            // TODO: handle better than just returning here!
            cerr << "Error in FreespaceDelaunayAlgorithm::moveVertex(): Attempted to move a vertex to an already existing vertex location" << endl;
            return;
        }

        // Get the cells that conflict in a vector vecConflictCells, and a facet on the boundary of this hole in f.
        vector<Delaunay3::Cell_handle> vecConflictCells;
        Delaunay3::Facet f;
        dt.find_conflicts(pd3NewPoint, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

        // Get the partitioned unioned constraint sets of all the cells in vecConflictCells.
        for (vector<Delaunay3::Cell_handle>::const_iterator it = vecConflictCells.begin(); it != vecConflictCells.end(); it++) {
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itConstraint = (*it)->info().getIntersections().begin();
                 itConstraint != (*it)->info().getIntersections().end(); itConstraint++) {
                if (itConstraint->second == vertexIndex)
                    setUnionedMovedConstraints.insert(*itConstraint);
                else
                    setUnionedStationaryConstraints.insert(*itConstraint);
            }
        }

        // Step 4
        hndlQ = dt.insert_in_hole(pd3NewPoint, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
        vecVertexHandles[vertexIndex] = hndlQ;

        // Step 6
        for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin(); itCell != dt.finite_cells_end(); itCell++) {
            if (itCell->info().isNew())
                setNewCells.insert(itCell);
            // Linear search:
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itDelete = itCell->info().getIntersections().begin();
                 itDelete != itCell->info().getIntersections().end(); ) {
                if (itDelete->second == vertexIndex) {
                    // invalidates iterator, so careful about incrementing it:
                    set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itNext = itDelete;
                    itNext++;
                    itCell->info().removeIntersection(itDelete->first, itDelete->second, vecVertexHandles, getCamCenters());
                    itCell->info().decrementVoteCount();
                    itDelete = itNext;
                }
                else
                    itDelete++;
            }
        }

        // Step 7
        for (set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedStationaryConstraints.begin();
                itConstraint != setUnionedStationaryConstraints.end(); itConstraint++) {
            Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
                                 PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[itConstraint->second], QO,
                                                            itConstraint->first, itConstraint->second, true);
        }
        for (set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedMovedConstraints.begin();
                itConstraint != setUnionedMovedConstraints.end(); itConstraint++) {
            Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
                                 PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[itConstraint->second], QO,
                                                            itConstraint->first, itConstraint->second, false);
        }
        for (set<Delaunay3::Cell_handle>::iterator itCell = setNewCells.begin(); itCell != setNewCells.end(); itCell++)
            (*itCell)->info().markOld();
    }

    void FreespaceDelaunayAlgorithm::moveVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const vector<int> & arrPointIndices) const {
        // Vertex Moving Algorithm:
        // ~~~~~~~~~~~~~~~~~~~~
        // Step 1: Collect FS constraints into two unioned sets from incident cells.  FS constraints containing the vertices to be moved go to their own set.
        //				(Since they will refer to a new point locations)
        // Step 2: Delete the vertices (this retriangulates).
        // Note: Steps 3 and 4 should be done as one combined step as the sets in step 3 make no sense without point insertion in between.
        // Step 3: As in point insertion, find the set of Delaunay-conflicting cells for the moved points, and add their FS constraints to the unioned sets.
        //				(Again same division between sets)
        // Step 4: Insert the points into the triangulation (this retriangulates).
        // Step 5: (nevermind...)  Used to be a "repeat steps 1-4" for each vertex
        // Step 6: Iterate over all cells in the DT and remove any FS constraints containing the deleted vertices.  Meanwhile determine the set of new cells.
        // Step 7: Process the FS constraints in the unioned sets.  Normal constraints only mark new cells.  Special constraints containing the moved
        //				points mark all crossed cells (as they were explicitly deleted in step 6).  Mark new cells as old.

        vector<Delaunay3::Vertex_handle> arrHndlQ;
        vector<int> arrVertexIndices;
        vector<PointD3> arrPd3NewPoints;
        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setUnionedStationaryConstraints;
        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setUnionedMovedConstraints;
        set<Delaunay3::Cell_handle> setNewCells;

        // TODO: DEBUG: PTAM has minor data corruption bugs, and this hack handles the bad data being passed to our code.  Should fix PTAM instead.
        for (vector<int>::const_iterator it = arrPointIndices.begin(); it != arrPointIndices.end(); it++) {
            int vertexIndex = m_mapPoint_VertexHandle[*it];
            Delaunay3::Vertex_handle hndlQ = vecVertexHandles[vertexIndex];
            if (! dt.is_vertex(hndlQ))
                continue;

            arrVertexIndices.push_back(vertexIndex);
            arrHndlQ.push_back(hndlQ);
            arrPd3NewPoints.push_back(PointD3(getPoint(*it)(0), getPoint(*it)(1), getPoint(*it)(2)));
        }
        if (arrVertexIndices.size() == 0)
            return;

        // Step 1:
        set<Delaunay3::Cell_handle> setIncidentCells;
        for (vector<Delaunay3::Vertex_handle>::iterator itHndlQ = arrHndlQ.begin(); itHndlQ != arrHndlQ.end(); itHndlQ++)
            dt.incident_cells(*itHndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

        for (set<Delaunay3::Cell_handle>::iterator itCell = setIncidentCells.begin(); itCell != setIncidentCells.end(); itCell++) {
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itConstraint = (*itCell)->info().getIntersections().begin();
                 itConstraint != (*itCell)->info().getIntersections().end(); itConstraint++) {
                if (std::find(arrVertexIndices.begin(), arrVertexIndices.end(), itConstraint->second) != arrVertexIndices.end()) // linear search in std::find()
                    setUnionedMovedConstraints.insert(*itConstraint);
                else
                    setUnionedStationaryConstraints.insert(*itConstraint);
            }
        }

        // Step 2:
        for (int nLoop = 0; nLoop < (int) arrHndlQ.size(); nLoop++) {
            Delaunay3::Vertex_handle hndlQ = arrHndlQ[nLoop];
            if (! dt.is_vertex(hndlQ))
                continue;
            dt.remove(hndlQ);
        }

        // Steps 3 & 4:

        for (int nLoop = 0; nLoop < (int)arrHndlQ.size(); nLoop++) {
            // Locate the point
            Delaunay3::Locate_type lt;
            int li, lj;
            Delaunay3::Cell_handle c = dt.locate(arrPd3NewPoints[nLoop], lt, li, lj);
            if (lt == Delaunay3::VERTEX) {
                // TODO: handle better than just returning here!
                cerr << "Error in FreespaceDelaunayAlgorithm::moveVertex(): Attempted to move a vertex to an already existing vertex location" << endl;
                return;
            }

            // Get the cells that conflict in a vector vecConflictCells, and a facet on the boundary of this hole in f.
            vector<Delaunay3::Cell_handle> vecConflictCells;
            Delaunay3::Facet f;
            dt.find_conflicts(arrPd3NewPoints[nLoop], c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

            // Get the partitioned unioned constraint sets of all the cells in vecConflictCells.
            for (vector<Delaunay3::Cell_handle>::const_iterator it = vecConflictCells.begin(); it != vecConflictCells.end(); it++) {
                for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itConstraint = (*it)->info().getIntersections().begin();
                     itConstraint != (*it)->info().getIntersections().end(); itConstraint++) {
                    if (std::find(arrVertexIndices.begin(), arrVertexIndices.end(), itConstraint->second) != arrVertexIndices.end()) // linear search in std::find()
                        setUnionedMovedConstraints.insert(*itConstraint);
                    else
                        setUnionedStationaryConstraints.insert(*itConstraint);
                }
            }

            // Step 4's stuff:
            arrHndlQ[nLoop] = dt.insert_in_hole(arrPd3NewPoints[nLoop], vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
            vecVertexHandles[arrVertexIndices[nLoop]] = arrHndlQ[nLoop];
        }

        // Step 6
        for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin(); itCell != dt.finite_cells_end(); itCell++) {
            if (itCell->info().isNew())
                setNewCells.insert(itCell);
            // Linear search:
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itDelete = itCell->info().getIntersections().begin();
                 itDelete != itCell->info().getIntersections().end(); ) {
                if (std::find(arrVertexIndices.begin(), arrVertexIndices.end(), itDelete->second) != arrVertexIndices.end()) { // linear search in std::find()
                    // invalidates iterator, so careful about incrementing it:
                    set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itNext = itDelete;
                    itNext++;
                    itCell->info().removeIntersection(itDelete->first, itDelete->second, vecVertexHandles, getCamCenters());
                    itCell->info().decrementVoteCount();
                    itDelete = itNext;
                }
                else
                    itDelete++;
            }
        }

        // Step 7
        for (set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedStationaryConstraints.begin();
                itConstraint != setUnionedStationaryConstraints.end(); itConstraint++) {
            Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
                                 PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[itConstraint->second], QO,
                                                            itConstraint->first, itConstraint->second, true);
        }
        for (set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedMovedConstraints.begin();
                itConstraint != setUnionedMovedConstraints.end(); itConstraint++) {
            Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
                                 PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[itConstraint->second], QO,
                                                            itConstraint->first, itConstraint->second, false);
        }
        for (set<Delaunay3::Cell_handle>::iterator itCell = setNewCells.begin(); itCell != setNewCells.end(); itCell++)
            (*itCell)->info().markOld();
    }

    void FreespaceDelaunayAlgorithm::applyConstraint(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int camIndex, const int pointIndex) const {
        int vertexIndex = m_mapPoint_VertexHandle[pointIndex];

        // TODO: DEBUG: PTAM has minor data corruption bugs, and this hack handles the bad data being passed to our code.  Should fix PTAM instead.
        if (! dt.is_vertex(vecVertexHandles[vertexIndex]))
            return;

        Segment QO = Segment(vecVertexHandles[vertexIndex]->point(), PointD3(getCamCenter(camIndex)(0), getCamCenter(camIndex)(1), getCamCenter(camIndex)(2)));
        markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[vertexIndex], QO, camIndex, vertexIndex, false);
    }

    void FreespaceDelaunayAlgorithm::removeConstraint(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int camIndex, const int pointIndex) const {
        int vertexIndex = m_mapPoint_VertexHandle[pointIndex];
        // TODO: DEBUG: PTAM has minor data corruption bugs, and this hack handles the bad data being passed to our code.  Should fix PTAM instead.
        if (! dt.is_vertex(vecVertexHandles[vertexIndex]))
            return;

        for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin(); itCell != dt.finite_cells_end(); itCell++) {
            // Linear search:
            for (set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itDelete = itCell->info().getIntersections().begin();
                 itDelete != itCell->info().getIntersections().end(); ) {
                if ((itDelete->first == camIndex) && (itDelete->second == vertexIndex)) {
                    // invalidates iterator, so careful about incrementing it:
                    set<Delaunay3CellInfo::FSConstraint, Delaunay3CellInfo::LtFSConstraint>::const_iterator itNext = itDelete;
                    itNext++;
                    itCell->info().removeIntersection(itDelete->first, itDelete->second, vecVertexHandles, getCamCenters());
                    itCell->info().decrementVoteCount();
                    itDelete = itNext;
                }
                else
                    itDelete++;
            }
        }
    }

    void FreespaceDelaunayAlgorithm::tetsToTris(const Delaunay3 & dt, vector<Matrix> & points, list<Matrix> & tris, const int nVoteThresh) const {
        // NEW Version, graph cut isosurf extraction with maxflow (builds the graph from scratch every time):
        {
            // TODO: Remove timing output for graphcuts.
            //cerr << "Running Graph Cut Isosurface Extraction..." << endl;
            //double t = timestamp();
            tetsToTris_maxFlowSimple(dt, points, tris, nVoteThresh);
            //cerr << "Time Taken (Isosurface): " << (timestamp() - t) << " s" << endl;
        }

        // OLD Version, simple isosurf extraction:
        // tetsToTris_naive(dt, points, tris, nVoteThresh);
    }

    int FreespaceDelaunayAlgorithm::writeObj(const string filename, const vector<Matrix> & points, const list<Matrix> & tris) const {
        // TODO: handle better for invalid files (e.g. throw exception).
        ofstream outfile;

        // Open file
        outfile.open(filename.c_str());
        if (! outfile.is_open()) {
            cerr << "Unable to open file: " << filename << endl;
            return -1;
        }

        // Write out lines one by one.
        for (vector<Matrix>::const_iterator itPoints = points.begin(); itPoints != points.end(); itPoints++)
            outfile << "v " << itPoints->at(0) << " " << itPoints->at(1) << " " << itPoints->at(2) << endl;
        for (list<Matrix>::const_iterator itTris = tris.begin(); itTris != tris.end(); itTris++)
            outfile << "f " << (round(itTris->at(0)) + 1) << " " << (round(itTris->at(1)) + 1) << " " << (round(itTris->at(2)) + 1) << endl;

        // Close the file and return
        outfile.close();
        return 0;
    }

    void FreespaceDelaunayAlgorithm::writeObj(ostream & outfile, const vector<Matrix> & points, const list<Matrix> & tris) const {
        // Write out lines one by one.
        for (vector<Matrix>::const_iterator itPoints = points.begin(); itPoints != points.end(); itPoints++)
            outfile << "v " << itPoints->at(0) << " " << itPoints->at(1) << " " << itPoints->at(2) << endl;
        for (list<Matrix>::const_iterator itTris = tris.begin(); itTris != tris.end(); itTris++)
            outfile << "f " << (round(itTris->at(0)) + 1) << " " << (round(itTris->at(1)) + 1) << " " << (round(itTris->at(2)) + 1) << endl;
    }

    // Private Methods

    void FreespaceDelaunayAlgorithm::copy(const vector<Matrix> & points, const vector<Matrix> & cams, const vector<Matrix> & camCenters,
                                          const vector<Matrix> & principleRays, const vector<vector<int> > & visibilityList) {
        m_points = points;
        m_cams = cams;
        m_camCenters = camCenters;
        m_principleRays = principleRays;
        m_visibilityList = visibilityList;
    }

    void FreespaceDelaunayAlgorithm::calculateBoundsValues() {
//    m_nBoundsMin = -100;
//    m_nBoundsMax = 100;
        m_nBoundsMin = -2;
        m_nBoundsMax = 2;
        for (int i = 0; i < numPoints(); i++) {
            if (getPoint(i)(0) > m_nBoundsMax)
                m_nBoundsMax = getPoint(i)(0);
            if (getPoint(i)(0) < m_nBoundsMin)
                m_nBoundsMin = getPoint(i)(0);
            if (getPoint(i)(1) > m_nBoundsMax)
                m_nBoundsMax = getPoint(i)(1);
            if (getPoint(i)(1) < m_nBoundsMin)
                m_nBoundsMin = getPoint(i)(1);
            if (getPoint(i)(2) > m_nBoundsMax)
                m_nBoundsMax = getPoint(i)(2);
            if (getPoint(i)(2) < m_nBoundsMin)
                m_nBoundsMin = getPoint(i)(2);
        }
        for (int i = 0; i < numCams(); i++) {
            if (getCamCenter(i)(0) > m_nBoundsMax)
                m_nBoundsMax = getCamCenter(i)(0);
            if (getCamCenter(i)(0) < m_nBoundsMin)
                m_nBoundsMin = getCamCenter(i)(0);
            if (getCamCenter(i)(1) > m_nBoundsMax)
                m_nBoundsMax = getCamCenter(i)(1);
            if (getCamCenter(i)(1) < m_nBoundsMin)
                m_nBoundsMin = getCamCenter(i)(1);
            if (getCamCenter(i)(2) > m_nBoundsMax)
                m_nBoundsMax = getCamCenter(i)(2);
            if (getCamCenter(i)(2) < m_nBoundsMin)
                m_nBoundsMin = getCamCenter(i)(2);
        }
        m_nBoundsMin *= 10.0;
        m_nBoundsMax *= 10.0;
    }

    void FreespaceDelaunayAlgorithm::createBounds(Delaunay3 & dt) const {
        // Note: The two values below worked with arbitrary precision arithmetic (though slowed down the execution),
        // but caused numerical difficulties in some computational geometry algorithms when using double-precision
        // const double nLargePosDouble = (double) numeric_limits<float>::max();
        // const double nLargeNegDouble = - (double) numeric_limits<float>::max();
        const double nLargePosDouble = getBoundsMax();
        const double nLargeNegDouble = getBoundsMin();

        // Insert 8 points forming an axis-aligned bounding box around our real data points.  Comments of + or - on the following lines indicates the limits of x y z, resp.,
        // that the corresponding corner point is at.
        dt.insert(PointD3(nLargePosDouble, nLargePosDouble, nLargePosDouble)); // + + +
        dt.insert(PointD3(nLargePosDouble, nLargePosDouble, nLargeNegDouble)); // + + -
        dt.insert(PointD3(nLargePosDouble, nLargeNegDouble, nLargePosDouble)); // + - +
        dt.insert(PointD3(nLargePosDouble, nLargeNegDouble, nLargeNegDouble)); // + - -
        dt.insert(PointD3(nLargeNegDouble, nLargePosDouble, nLargePosDouble)); // - + +
        dt.insert(PointD3(nLargeNegDouble, nLargePosDouble, nLargeNegDouble)); // - + -
        dt.insert(PointD3(nLargeNegDouble, nLargeNegDouble, nLargePosDouble)); // - - +
        dt.insert(PointD3(nLargeNegDouble, nLargeNegDouble, nLargeNegDouble)); // - - -
    }

    void FreespaceDelaunayAlgorithm::markTetrahedraCrossingConstraint(Delaunay3 & dt, const Delaunay3::Vertex_handle hndlQ, const Segment & constraint) const {
        Delaunay3::Cell_handle tetPrev;
        Delaunay3::Cell_handle tetCur;
        Delaunay3::Locate_type lt; int li, lj;

        Matrix matQ(3, 1, 1.0);
        Matrix matO(3, 1, 1.0);
        matQ(0) = constraint.source().x();
        matQ(1) = constraint.source().y();
        matQ(2) = constraint.source().z();
        matO(0) = constraint.target().x();
        matO(1) = constraint.target().y();
        matO(2) = constraint.target().z();

        // For all tetrahedra t incident to Q:
        vector<Delaunay3::Cell_handle> vecQCells;
        dt.incident_cells(hndlQ, std::back_inserter(vecQCells));
        vector<Delaunay3::Cell_handle>::iterator itQCells;
        for (itQCells = vecQCells.begin(); itQCells != vecQCells.end(); itQCells++) {
            // If t contains O:
            if (dt.side_of_cell(constraint.target(), *itQCells, lt, li, lj) != CGAL::ON_UNBOUNDED_SIDE) {
                (*itQCells)->info().incrementVoteCount(); // t.n++
                // We're done, so return
                return;
            }
            // Let f be the facet of t opposite to Q
            int f = (*itQCells)->index(hndlQ); // this with the Cell_handle *itQCells defines the facet
            // If f intersects QO
            //if (CGAL::do_intersect(dt.triangle(*itQCells, f), constraint)) {
            if (triangleConstraintIntersectionTest(Delaunay3::Facet(*itQCells, f), matQ, matO)) {
                tetPrev = *itQCells; // t.precedent = t
                tetCur = (*itQCells)->neighbor(f); // t.actual = neighbour of t incident to facet f
                (*itQCells)->info().incrementVoteCount(); // t.n++
                tetCur->info().incrementVoteCount(); // t.actual.n++
                break;
            }
        }
        // While t.actual doesn't contain O
        int f;
        //while (dt.side_of_cell(constraint.target(), tetCur, lt, li, lj) == CGAL::ON_UNBOUNDED_SIDE) {
        while (cellTraversalExitTest(f, tetCur, tetPrev, matQ, matO)) {
            // f is now the facet of t.actual that intersects QO and that isn't incident to t.precedent

            //for (f = 0; f < 4; f++) {
            //  if (tetCur->neighbor(f) == tetPrev) continue;
            //  if (CGAL::do_intersect(dt.triangle(tetCur, f), constraint)) break;
            //}

            tetPrev = tetCur; // t.precedent = t.actual
            tetCur = tetCur->neighbor(f); // t.actual = neighbour of t.precedent(==t.actual) incident to facet f
            tetCur->info().incrementVoteCount(); // t.actual.n++
        }

        //	 We find intersecting tetrahedra by Pau's algorithm:
        //	 let Q be the point & O the optic center.
        //	 For all tetrahedra t incident to Q:
        //		if t contains O:
        //			t.n++
        //			break maybe?  bad translation.  In this case should be ~done~ here anyways.
        //		let f be the facet	 of t opposite to Q
        //		if f intersects QO:
        //			t.precedent = t
        //			t.actual = neighbour of t incident to facet f
        //			t.n++;
        //			t.actual.n++;
        //			break from for loop
        //	 while t.actual doesn't contain O
        //		let f be the facet of t.actual that intersects QO and which isn't incident to t.precedent
        //		t.precedent = t.actual
        //		t.actual = neighbour of t.precedent(==t.actual) incident to facet f
        //		t.actual.n++;

        // Basically Pau's algorithm says:
        // Traverse the tetrahedrization.  For all traversed tetrahedra, increment the count.
        // Start with the tetrahedron that contains Q.  Do this by the algorithm of localization of a point in a delaunay triangulization.
        // (Here, this is modified by use of a triangulation hierarchy from CGAL for fast point localization.)
        // Loop over all neighbouring tetrahedra and find the one whose common facet intersects the segment QO.  Traverse to this.
        // Continue traversing adjacent tetrahedra by the constraint-intersecting facets.
        // Stop traversal when O is contained in the current tetrahedron.
    }

    void FreespaceDelaunayAlgorithm::markTetrahedraCrossingConstraintWithBookKeeping(Delaunay3 & dt, const vector<Delaunay3::Vertex_handle> & vecVertexHandles,
                                                                                     const Delaunay3::Vertex_handle hndlQ, const Segment & constraint, const int camIndex, const int featureIndex, const bool bOnlyMarkNew) const {
        Delaunay3::Cell_handle tetPrev;
        Delaunay3::Cell_handle tetCur;
        Delaunay3::Locate_type lt; int li, lj;

        Matrix matQ(3, 1);
        Matrix matO(3, 1);
        matQ(0) = constraint.source().x();
        matQ(1) = constraint.source().y();
        matQ(2) = constraint.source().z();
        matO(0) = constraint.target().x();
        matO(1) = constraint.target().y();
        matO(2) = constraint.target().z();

        // For all tetrahedra t incident to Q:
        vector<Delaunay3::Cell_handle> vecQCells;
        dt.incident_cells(hndlQ, std::back_inserter(vecQCells));
        vector<Delaunay3::Cell_handle>::iterator itQCells;
        for (itQCells = vecQCells.begin(); itQCells != vecQCells.end(); itQCells++) {
            // If t contains O:
            if (dt.side_of_cell(constraint.target(), *itQCells, lt, li, lj) != CGAL::ON_UNBOUNDED_SIDE) {
                if (! bOnlyMarkNew || (*itQCells)->info().isNew()) {
                    (*itQCells)->info().incrementVoteCount(); // t.n++
                    (*itQCells)->info().addIntersection(camIndex, featureIndex, vecVertexHandles, getCamCenters());
                }
                // We're done, so return
                return;
            }
            // Let f be the facet of t opposite to Q
            int f = (*itQCells)->index(hndlQ); // this with the Cell_handle *itQCells defines the facet
            // If f intersects QO
            if (CGAL::do_intersect(dt.triangle(*itQCells, f), constraint)) {
                //if (triangleConstraintIntersectionTest(Delaunay3::Facet(*itQCells, f), matQ, matO)) {
                tetPrev = *itQCells; // t.precedent = t
                tetCur = (*itQCells)->neighbor(f); // t.actual = neighbour of t incident to facet f
                if (! bOnlyMarkNew || (*itQCells)->info().isNew()) {
                    (*itQCells)->info().incrementVoteCount(); // t.n++
                    (*itQCells)->info().addIntersection(camIndex, featureIndex, vecVertexHandles, getCamCenters());
                }
                if (! bOnlyMarkNew || tetCur->info().isNew()) {
                    tetCur->info().incrementVoteCount(); // t.actual.n++
                    tetCur->info().addIntersection(camIndex, featureIndex, vecVertexHandles, getCamCenters());
                }
                break;
            }
        }
        // While t.actual doesn't contain O
        int f;

        //while(dt.side_of_cell(constraint.target(), tetCur, lt, li, lj) == CGAL::ON_UNBOUNDED_SIDE){
        while (cellTraversalExitTest(f, tetCur, tetPrev, matQ, matO)) {
            // f is now the facet of t.actual that intersects QO and that isn't incident to t.precedent

            //for (f = 0; f < 4; f++) {
            //  if (tetCur->neighbor(f) == tetPrev) continue;
            //  if (CGAL::do_intersect(dt.triangle(tetCur, f), constraint)) break;
            //}

            tetPrev = tetCur; // t.precedent = t.actual
            tetCur = tetCur->neighbor(f); // t.actual = neighbour of t.precedent(==t.actual) incident to facet f
            if (! bOnlyMarkNew || tetCur->info().isNew()) {
                tetCur->info().incrementVoteCount(); // t.actual.n++
                tetCur->info().addIntersection(camIndex, featureIndex, vecVertexHandles, getCamCenters());
            }
        }

        //	 We find intersecting tetrahedra by Pau's algorithm:
        //	 let Q be the point & O the optic center.
        //	 For all tetrahedra t incident to Q:
        //		if t contains O:
        //			t.n++
        //			break maybe?  bad translation.  In this case should be ~done~ here anyways.
        //		let f be the facet	 of t opposite to Q
        //		if f intersects QO:
        //			t.precedent = t
        //			t.actual = neighbour of t incident to facet f
        //			t.n++;
        //			t.actual.n++;
        //			break from for loop
        //	 while t.actual doesn't contain O
        //		let f be the facet of t.actual that intersects QO and which isn't incident to t.precedent
        //		t.precedent = t.actual
        //		t.actual = neighbour of t.precedent(==t.actual) incident to facet f
        //		t.actual.n++;

        // Basically Pau's algorithm says:
        // Traverse the tetrahedrization.  For all traversed tetrahedra, increment the count.
        // Start with the tetrahedron that contains Q.  Do this by the algorithm of localization of a point in a delaunay triangulization.
        // (Here, this is modified by use of a triangulation hierarchy from CGAL for fast point localization.)
        // Loop over all neighbouring tetrahedra and find the one whose common facet intersects the segment QO.  Traverse to this.
        // Continue traversing adjacent tetrahedra by the constraint-intersecting facets.
        // Stop traversal when O is contained in the current tetrahedron.
    }

    void FreespaceDelaunayAlgorithm::addNewlyObservedFeatures(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles,
                                                              vector<int> & localVisList, const vector<int> & originalLocalVisList) const {

        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setUnionedConstraints;
        int oldNumVertices = (int)vecVertexHandles.size();

        // Add new features to the global model that weren't observed before, and construct the local visibility list:
        vector<int>::const_iterator itOriginalLocalVisList;
        int i;
        for (itOriginalLocalVisList = originalLocalVisList.begin(); itOriginalLocalVisList != originalLocalVisList.end(); itOriginalLocalVisList++) {
            // OLD CODE: no point index -> vertex index map
            /*int originalIndex = *itOriginalLocalVisList;
            Matrix matTmpPoint = getPoint(originalIndex);
            PointD3 pd3TmpPoint(matTmpPoint(0), matTmpPoint(1), matTmpPoint(2));

            // TODO: consider whether it's more efficient to use dt.locate() instead of iterating through vecVertexHandles below.
              // (Will still need to iterate for some vertices, or perhaps just keep a reference in each vertex to get the proper index.)

            for(i = 0; i < (int)vecVertexHandles.size(); i++){
              if(doubleEquals(vecVertexHandles[i]->point().x(), matTmpPoint(0)) && doubleEquals(vecVertexHandles[i]->point().y(), matTmpPoint(1))
                  && doubleEquals(vecVertexHandles[i]->point().z(), matTmpPoint(2))){
                // We found a match: this feature is already in the point set.  So add it to the visibility list and break.
                localVisList.push_back(i);
                break;
              }
            }
            if(i == (int)vecVertexHandles.size()){
              // We didn't find a match, so this is a new feature.  Add it to the point set and the visibility list.
              // This involves properly deleting & staring off a connected subset of tetrahedra that violate the Delaunay constraint
              // after the addition, while marking the new tetrahedra with the deleted tetrahedra's freespace constraints.
              addNewlyObservedFeature(dt, vecVertexHandles, setUnionedConstraints, pd3TmpPoint);
              localVisList.push_back(i);
            }*/

            // NEW CODE: with point index -> vertex index map
            int originalIndex = *itOriginalLocalVisList;

            map<int, int>::iterator itVertHandleIndex = m_mapPoint_VertexHandle.find(originalIndex);
            if (itVertHandleIndex != m_mapPoint_VertexHandle.end()) {
                // We found a match: this feature is already in the point set.  So add it to the visibility list and break.
                localVisList.push_back(itVertHandleIndex->second);
            }
            else {
                // We didn't find a match, so this is a new feature.  Add it to the point set and the visibility list.
                // This involves properly deleting & staring off a connected subset of tetrahedra that violate the Delaunay constraint
                // after the addition, while marking the new tetrahedra with the deleted tetrahedra's freespace constraints.
                Matrix matTmpPoint = getPoint(originalIndex);
                PointD3 pd3TmpPoint(matTmpPoint(0), matTmpPoint(1), matTmpPoint(2));
                addNewlyObservedFeature(dt, vecVertexHandles, setUnionedConstraints, pd3TmpPoint, originalIndex);
                localVisList.push_back((int)vecVertexHandles.size() - 1);
            }
        }

        // Apply the unioned constraint set to all the new cells adjacent to all the new points Q.  Increment the vote counts.
        // ::: OLD METHOD :::
        /*vector<Delaunay3::Cell_handle> vecNewCells;
        dt.incident_cells(hndlQ, std::back_inserter(vecNewCells));
        for(set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedConstraints.begin(); itConstraint != setUnionedConstraints.end(); itConstraint++){
          Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
            PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
          for(vector<Delaunay3::Cell_handle>::iterator itCell = vecNewCells.begin(); itCell != vecNewCells.end(); itCell++){
            if(cellConstraintIntersectionTest(dt, *itCell, QO)){
              // Add the constraint to itCell and increment its vote count.
              (*itCell)->info().incrementVoteCount();
              (*itCell)->info().addIntersection(itConstraint->first, itConstraint->second);
            }
          }
        }*/
        // ::: NEW METHOD :::
        set<Delaunay3::Cell_handle> setNewCells;
        for (i = oldNumVertices; i < (int)vecVertexHandles.size(); i++)
            dt.incident_cells(vecVertexHandles[i], std::inserter(setNewCells, setNewCells.begin()));
        for (set<pair<int, int>, Delaunay3CellInfo::LtConstraint>::iterator itConstraint = setUnionedConstraints.begin(); itConstraint != setUnionedConstraints.end(); itConstraint++) {
            Segment QO = Segment(vecVertexHandles[itConstraint->second]->point(),
                                 PointD3(getCamCenter(itConstraint->first)(0), getCamCenter(itConstraint->first)(1), getCamCenter(itConstraint->first)(2)));
            markTetrahedraCrossingConstraintWithBookKeeping(dt, vecVertexHandles, vecVertexHandles[itConstraint->second], QO, itConstraint->first, itConstraint->second, true);
        }
        for (set<Delaunay3::Cell_handle>::iterator itCell = setNewCells.begin(); itCell != setNewCells.end(); itCell++)
            (*itCell)->info().markOld();

        // Runtime Analysis Output
        /*cerr << endl;
        cerr << "# of vertices added: " << ((int)vecVertexHandles.size() - oldNumVertices) << endl;
        cerr << "# of new tetrahedra after retriangulation: " << setNewCells.size() << endl;
        cerr << "# of all tetrahedra after retriangulation: " << dt.number_of_finite_cells() << endl;
        cerr << "# of constraints from deleted tetrahedra (doesn't include this view's new observations): " << setUnionedConstraints.size() << endl;
        cerr << "# of constraints in this triangulation (doesn't include this view's new observations): ";
        set<pair<int, int>, Delaunay3CellInfo::LtConstraint> setAllConstraints;
        for(Delaunay3::Finite_cells_iterator itAllCells = dt.finite_cells_begin(); itAllCells != dt.finite_cells_end(); itAllCells++)
          setAllConstraints.insert(itAllCells->info().getIntersections().begin(), itAllCells->info().getIntersections().end());
        cerr << setAllConstraints.size() << endl;*/
    }

    void FreespaceDelaunayAlgorithm::addNewlyObservedFeature(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles,
                                                             set<pair<int, int>, Delaunay3CellInfo::LtConstraint> & setUnionedConstraints, const PointD3 & Q, const int nPointIndex) const {
        // Add the point Q to the triangulated point set dt.
        // This involves properly deleting & staring off a connected subset of tetrahedra that violate the Delaunay constraint
        // after the addition, while marking the new tetrahedra with the deleted tetrahedra's freespace constraints.

        // Locate the point
        Delaunay3::Locate_type lt;
        int li, lj;
        Delaunay3::Cell_handle c = dt.locate(Q, lt, li, lj);
        if (lt == Delaunay3::VERTEX) {
            cerr << "Error in FreespaceDelaunayAlgorithm::addNewlyObservedFeature(): Attempted to add a duplicate vertex to the triangulation" << endl;
            return;
        }

        // Get the cells that conflict with Q in a vector vecConflictCells, and a facet on the boundary of this hole in f.
        vector<Delaunay3::Cell_handle> vecConflictCells;
        Delaunay3::Facet f;
        dt.find_conflicts(Q, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

        // Get the unioned constraint set of all the cells in vecConflictCells.
        for (vector<Delaunay3::Cell_handle>::const_iterator it = vecConflictCells.begin(); it != vecConflictCells.end(); it++)
            setUnionedConstraints.insert((*it)->info().getIntersections().begin(), (*it)->info().getIntersections().end());

        // Delete the cells in conflict, insert the point, and star off the hole.
        Delaunay3::Vertex_handle hndlQ = dt.insert_in_hole(Q, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);

        // Add Q's Vertex_handle to vecVertexHandles
        vecVertexHandles.push_back(hndlQ);
        m_mapPoint_VertexHandle[nPointIndex] = (int)vecVertexHandles.size() - 1;
    }

    bool FreespaceDelaunayAlgorithm::triangleConstraintIntersectionTest(const Delaunay3::Facet & tri, const Matrix & segSrc, const Matrix & segDest) const {
        // A custom implementation of the ray-triangle intersection test at http://jgt.akpeters.com/papers/MollerTrumbore97/code.html
        // Follows the back-face culling branch (ie: a triangle won't intersect the ray if the ray pierces the backside of it.)
        Matrix edge1, edge2, tvec, pvec, qvec;
        double det, inv_det;
        double t, u, v;

        // Get the 3 triangle vertices
        Matrix v0(3, 1);
        Matrix v1(3, 1);
        Matrix v2(3, 1);

        // Note:
        // tri.first = the Cell_handle containing the triangle
        // tri.second = f, the face index for tri.first
        // We want all normals pointing inward, ie positive halfspace of a triangle contains the 4th point of the tetrahedron.  So:
        // f == 0 -> (1, 3, 2)
        // f == 1 -> (0, 2, 3)
        // f == 2 -> (3, 1, 0)
        // f == 3 -> (0, 1, 2)
        if (tri.second == 3) {
            v0(0) = tri.first->vertex(0)->point().x(); v0(1) = tri.first->vertex(0)->point().y(); v0(2) = tri.first->vertex(0)->point().z();
            v1(0) = tri.first->vertex(1)->point().x(); v1(1) = tri.first->vertex(1)->point().y(); v1(2) = tri.first->vertex(1)->point().z();
            v2(0) = tri.first->vertex(2)->point().x(); v2(1) = tri.first->vertex(2)->point().y(); v2(2) = tri.first->vertex(2)->point().z();
        }
        else if (tri.second == 2) {
            v0(0) = tri.first->vertex(3)->point().x(); v0(1) = tri.first->vertex(3)->point().y(); v0(2) = tri.first->vertex(3)->point().z();
            v1(0) = tri.first->vertex(1)->point().x(); v1(1) = tri.first->vertex(1)->point().y(); v1(2) = tri.first->vertex(1)->point().z();
            v2(0) = tri.first->vertex(0)->point().x(); v2(1) = tri.first->vertex(0)->point().y(); v2(2) = tri.first->vertex(0)->point().z();
        }
        else if (tri.second == 1) {
            v0(0) = tri.first->vertex(0)->point().x(); v0(1) = tri.first->vertex(0)->point().y(); v0(2) = tri.first->vertex(0)->point().z();
            v1(0) = tri.first->vertex(2)->point().x(); v1(1) = tri.first->vertex(2)->point().y(); v1(2) = tri.first->vertex(2)->point().z();
            v2(0) = tri.first->vertex(3)->point().x(); v2(1) = tri.first->vertex(3)->point().y(); v2(2) = tri.first->vertex(3)->point().z();
        }
        else if (tri.second == 0) { // f == 0
            v0(0) = tri.first->vertex(1)->point().x(); v0(1) = tri.first->vertex(1)->point().y(); v0(2) = tri.first->vertex(1)->point().z();
            v1(0) = tri.first->vertex(3)->point().x(); v1(1) = tri.first->vertex(3)->point().y(); v1(2) = tri.first->vertex(3)->point().z();
            v2(0) = tri.first->vertex(2)->point().x(); v2(1) = tri.first->vertex(2)->point().y(); v2(2) = tri.first->vertex(2)->point().z();
        }
        else {
            cerr << "whaomg" << endl;
        }

        // Get the constraint ray's normalized direction vector
        Matrix dir = segDest - segSrc;
        double dirNorm = dir.norm();
        dir /= dirNorm;

        // Find vectors for two edges sharing v0:
        edge1 = v1 - v0;
        edge2 = v2 - v0;

        // Begin calculating determinant - also used to calculate U parameter
        pvec = dir.cross(edge2);

        // If determinant is near zero, ray lies in plane of triangle.  We do backface culling for the intersection test,
        // so only need to check 1 halfspace
        det = edge1.dot(pvec);
        if (det < sqrt_eps_d)
            return false;

        // Calculate distance from v0 to ray origin
        tvec = segSrc - v0;

        // Calculate U parameter and test bounds
        u = tvec.dot(pvec);
        if (u < 0.0 || u > det)
            return false;

        // Prepare to test V parameter
        qvec = tvec.cross(edge1);

        // Calculate V parameter and test bounds
        v = dir.dot(qvec);
        if (v < 0.0 || (u + v) > det)
            return false;

        // Calculate t, scale parameters, ray intersects triangle
        t = edge2.dot(qvec);
        inv_det = 1.0 / det;
        t *= inv_det;
        // u *= inv_det;
        // v *= inv_det;

        // Test if distance to plane t is too large and return false if so
        if (t < 0.0 || t > dirNorm)
            return false;

        return true;
    }

    bool FreespaceDelaunayAlgorithm::triangleConstraintIntersectionTest(bool & bCrossesInteriorOfConstraint, const vector<Matrix> & points, const Matrix & tri, const pair<Matrix, Matrix> & constraint) const {
        // A custom implementation of the ray-triangle intersection test at http://jgt.akpeters.com/papers/MollerTrumbore97/code.html
        // Follows the back-face culling branch (ie: a triangle won't intersect the ray if the ray pierces the backside of it.)
        Matrix edge1, edge2, tvec, pvec, qvec;
        double det, inv_det;
        double t, u, v;

        // Set default for interiorOfConstraint = false return value (in the case that there is no intersection for quick returns)
        bCrossesInteriorOfConstraint = false;

        // Get the two segment points:
        const Matrix & segSrc = constraint.first;
        const Matrix & segDest = constraint.second;

        // Get the 3 triangle vertices
        const Matrix & v0 = points[round(tri(0))];
        const Matrix & v1 = points[round(tri(1))];
        const Matrix & v2 = points[round(tri(2))];

        // Get the constraint ray's normalized direction vector
        Matrix dir = segDest - segSrc;
        double dirNorm = dir.norm();
        dir /= dirNorm;

        // Find vectors for two edges sharing v0:
        edge1 = v1 - v0;
        edge2 = v2 - v0;

        // Begin calculating determinant - also used to calculate U parameter
        pvec = dir.cross(edge2);

        // If determinant is near zero, ray lies in plane of triangle.  We do backface culling for the intersection test,
        // so only need to check 1 halfspace
        det = edge1.dot(pvec);
        if (det < sqrt_eps_d)
            return false;

        // Calculate distance from v0 to ray origin
        tvec = segSrc - v0;

        // Calculate U parameter and test bounds
        u = tvec.dot(pvec);
        if (u < 0.0 || u > det)
            return false;

        // Prepare to test V parameter
        qvec = tvec.cross(edge1);

        // Calculate V parameter and test bounds
        v = dir.dot(qvec);
        if (v < 0.0 || (u + v) > det)
            return false;

        // Calculate t, scale parameters, ray intersects triangle
        t = edge2.dot(qvec);
        inv_det = 1.0 / det;
        t *= inv_det;
        // u *= inv_det;
        // v *= inv_det;

        // Test if distance to plane t is too large and return false if so
        if (t < 0.0 || t > dirNorm)
            return false;
        if (t != 0.0 && t != dirNorm)
            bCrossesInteriorOfConstraint = true;

        return true;
    }

    bool FreespaceDelaunayAlgorithm::cellTraversalExitTest(int & f, const Delaunay3::Cell_handle tetCur, const Delaunay3::Cell_handle tetPrev, const Matrix & matQ,
                                                           const Matrix & matO) const {
        // TODO: See if we can optimize this by reuse: we use the same constraint QO in all 3 face tests.  Faces also share edges and points.
        vector<Matrix> points;
        Matrix tri(3, 1);
        pair<Matrix, Matrix> constraint(matQ, matO);
        bool bCrossesInteriorOfConstraint;
        Matrix tmpMat(3, 1);

        // Let f be the entry face's index.
        if (tetCur->neighbor(0) == tetPrev) f = 0;
        else if (tetCur->neighbor(1) == tetPrev) f = 1;
        else if (tetCur->neighbor(2) == tetPrev) f = 2;
        else if (tetCur->neighbor(3) == tetPrev) f = 3;

        // Collect the tetrahedra's 4 vertices into the variable points
        tmpMat(0) = tetCur->vertex(0)->point().x(); tmpMat(1) = tetCur->vertex(0)->point().y(); tmpMat(2) = tetCur->vertex(0)->point().z();
        points.push_back(tmpMat);
        tmpMat(0) = tetCur->vertex(1)->point().x(); tmpMat(1) = tetCur->vertex(1)->point().y(); tmpMat(2) = tetCur->vertex(1)->point().z();
        points.push_back(tmpMat);
        tmpMat(0) = tetCur->vertex(2)->point().x(); tmpMat(1) = tetCur->vertex(2)->point().y(); tmpMat(2) = tetCur->vertex(2)->point().z();
        points.push_back(tmpMat);
        tmpMat(0) = tetCur->vertex(3)->point().x(); tmpMat(1) = tetCur->vertex(3)->point().y(); tmpMat(2) = tetCur->vertex(3)->point().z();
        points.push_back(tmpMat);

        // Construct the indices for the triangles to test, and test them as needed.
        // We want all normals pointing inward, ie positive halfspace of a triangle contains the 4th point of the tetrahedron.  So:
        // f == 0 -> (1, 3, 2)
        // f == 1 -> (0, 2, 3)
        // f == 2 -> (3, 1, 0)
        // f == 3 -> (0, 1, 2)
        if (f == 3) {
            // Test face 0
            tri(0) = 1; tri(1) = 3; tri(2) = 2;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 0;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 1
            tri(0) = 0; tri(1) = 2; tri(2) = 3;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 1;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 2
            tri(0) = 3; tri(1) = 1; tri(2) = 0;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 2;
                return bCrossesInteriorOfConstraint;
            }
        }
        else if (f == 2) {
            // Test face 0
            tri(0) = 1; tri(1) = 3; tri(2) = 2;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 0;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 1
            tri(0) = 0; tri(1) = 2; tri(2) = 3;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 1;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 3
            tri(0) = 0; tri(1) = 1; tri(2) = 2;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 3;
                return bCrossesInteriorOfConstraint;
            }
        }
        else if (f == 1) {
            // Test face 0
            tri(0) = 1; tri(1) = 3; tri(2) = 2;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 0;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 2
            tri(0) = 3; tri(1) = 1; tri(2) = 0;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 2;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 3
            tri(0) = 0; tri(1) = 1; tri(2) = 2;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 3;
                return bCrossesInteriorOfConstraint;
            }
        }
        else { // f == 0
            // Test face 1
            tri(0) = 0; tri(1) = 2; tri(2) = 3;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 1;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 2
            tri(0) = 3; tri(1) = 1; tri(2) = 0;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 2;
                return bCrossesInteriorOfConstraint;
            }
            // Test face 3
            tri(0) = 0; tri(1) = 1; tri(2) = 2;
            if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
                f = 3;
                return bCrossesInteriorOfConstraint;
            }
        }
        // If no triangle intersects the constraint, then the optic center lies in the interior (or on the entry face) of the tetrahedron.  So return false.
        return false;
    }

    void FreespaceDelaunayAlgorithm::facetToTri(const Delaunay3::Facet & f, vector<Delaunay3::Vertex_handle> & vecTri) const {
        if (f.second == 0) {
            // Vertex handle order: 3 2 1
            vecTri.push_back(f.first->vertex(3));
            vecTri.push_back(f.first->vertex(2));
            vecTri.push_back(f.first->vertex(1));
        }
        else if (f.second == 1) {
            // Vertex handle order: 0 2 3
            vecTri.push_back(f.first->vertex(0));
            vecTri.push_back(f.first->vertex(2));
            vecTri.push_back(f.first->vertex(3));
        }
        else if (f.second == 2) {
            // Vertex handle order: 3 1 0
            vecTri.push_back(f.first->vertex(3));
            vecTri.push_back(f.first->vertex(1));
            vecTri.push_back(f.first->vertex(0));
        }
        else { // f->second == 3
            // Vertex handle order: 0 1 2
            vecTri.push_back(f.first->vertex(0));
            vecTri.push_back(f.first->vertex(1));
            vecTri.push_back(f.first->vertex(2));
        }
    }

    double FreespaceDelaunayAlgorithm::timestamp() const {
        timeval t;
        gettimeofday(&t, 0);
        return (double)(t.tv_sec + (t.tv_usec / 1000000.0));
    }

    void FreespaceDelaunayAlgorithm::tetsToTris_naive(const Delaunay3 & dt, vector<Matrix> & points, list<Matrix> & tris, const int nVoteThresh) const {
        vector<Delaunay3::Vertex_handle> vecBoundsHandles;
        vector<Delaunay3::Vertex_handle> vecVertexHandles;
        std::unordered_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;
        Matrix matTmpPoint(3, 1);

        // Initialize points and tris as empty:
        if (! points.empty()) points.clear();
        if (! tris.empty()) tris.clear();

        // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
        dt.incident_vertices (dt.infinite_vertex(), std::back_inserter(vecBoundsHandles));

        // Populate the model's point list, create a list of finite non-bounding vertex handles, and
        // create a useful associative maps (handle->point list index).
        for (Delaunay3::Finite_vertices_iterator itVert = dt.finite_vertices_begin(); itVert != dt.finite_vertices_end(); itVert++) {
            vector<Delaunay3::Vertex_handle>::iterator itBounds;
            for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
                if ((*itBounds) == ((Delaunay3::Vertex_handle)itVert))
                    break;
            }
            if (itBounds == vecBoundsHandles.end()) { // the vertex is not a bounding vertex, so add it
                matTmpPoint(0) = itVert->point().x();
                matTmpPoint(1) = itVert->point().y();
                matTmpPoint(2) = itVert->point().z();
                points.push_back(matTmpPoint);
                vecVertexHandles.push_back(itVert);
                hmapVertexHandleToIndex[itVert] = points.size() - 1;
            }
        }

        // Iterate over finite facets
        for (Delaunay3::Finite_facets_iterator itFacet = dt.finite_facets_begin(); itFacet != dt.finite_facets_end(); itFacet++) {
            // If one adjacent cell is empty, and the other is not, and if the facet contains no vertex from the bounding vertices, then add a triangle to the mesh (w/ correct orientation).
            bool bFacetCellKept = itFacet->first->info().isKeptByVoteCount(nVoteThresh);
            bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().isKeptByVoteCount(nVoteThresh);
            if (bFacetCellKept && ! bMirrorCellKept) {
                bool bContainsBoundsVert = false;
                for (int i = 0; i < 4; i++) {
                    if (i == itFacet->second) continue;
                    if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) { bContainsBoundsVert = true;  break; }
                }
                if (! bContainsBoundsVert) {
                    Delaunay3::Facet fTmp = dt.mirror_facet(*itFacet); // The normal points inward so mirror the facet
                    Matrix tmpTri(1, 3);
                    vector<Delaunay3::Vertex_handle> vecTri;

                    facetToTri(fTmp, vecTri);
                    tmpTri(0) = hmapVertexHandleToIndex[vecTri[0]];
                    tmpTri(1) = hmapVertexHandleToIndex[vecTri[1]];
                    tmpTri(2) = hmapVertexHandleToIndex[vecTri[2]];
                    tris.push_back(tmpTri);
                }
            }
            else if (bMirrorCellKept && ! bFacetCellKept) {
                bool bContainsBoundsVert = false;
                for (int i = 0; i < 4; i++) {
                    if (i == itFacet->second) continue;
                    if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) { bContainsBoundsVert = true;  break; }
                }
                if (! bContainsBoundsVert) {
                    Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
                    Matrix tmpTri(1, 3);
                    vector<Delaunay3::Vertex_handle> vecTri;

                    facetToTri(fTmp, vecTri);
                    tmpTri(0) = hmapVertexHandleToIndex[vecTri[0]];
                    tmpTri(1) = hmapVertexHandleToIndex[vecTri[1]];
                    tmpTri(2) = hmapVertexHandleToIndex[vecTri[2]];
                    tris.push_back(tmpTri);
                }
            }
        }
    }

    void FreespaceDelaunayAlgorithm::tetsToTris_maxFlowSimple(const Delaunay3 & dt, vector<Matrix> & points, list<Matrix> & tris, const int nVoteThresh) const {
        vector<Delaunay3::Vertex_handle> vecBoundsHandles;
        vector<Delaunay3::Vertex_handle> vecVertexHandles;
        std::unordered_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;
        map<Delaunay3::Cell_handle, int> mapCellHandleToIndex;
        Matrix matTmpPoint(3, 1);
        int loop;

        // Get some size-properties from the triangulation (non-constant-time access functions in the triangulation class)
        int numFiniteTets = dt.number_of_finite_cells();
        int numFiniteFacets = dt.number_of_finite_facets();

        // Initialize the maxflow graph
        Graph_t graph(numFiniteTets, numFiniteFacets);
        graph.addSource();
        graph.addSink();

        // Initialize points and tris as empty:
        if (! points.empty()) points.clear();
        if (! tris.empty()) tris.clear();

        // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
        dt.incident_vertices (dt.infinite_vertex(), std::back_inserter(vecBoundsHandles));

        // Populate the model's point list, create a list of finite non-bounding vertex handles, and
        // create a useful associative map (handle->point list index).
        for (Delaunay3::Finite_vertices_iterator itVert = dt.finite_vertices_begin(); itVert != dt.finite_vertices_end(); itVert++) {
            vector<Delaunay3::Vertex_handle>::iterator itBounds;
            for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
                if ((*itBounds) == ((Delaunay3::Vertex_handle)itVert))
                    break;
            }
            if (itBounds == vecBoundsHandles.end()) { // the vertex is not a bounding vertex, so add it
                matTmpPoint(0) = itVert->point().x();
                matTmpPoint(1) = itVert->point().y();
                matTmpPoint(2) = itVert->point().z();
                points.push_back(matTmpPoint);
                vecVertexHandles.push_back(itVert);
                hmapVertexHandleToIndex[itVert] = points.size() - 1;
            }
        }

        // Create useful associative maps (tet list index->handle & handle->tet list index).
        Delaunay3::Finite_cells_iterator it;
        for (loop = 0, it = dt.finite_cells_begin(); it != dt.finite_cells_end(); it++, loop++)
            mapCellHandleToIndex[it] = loop;

        // Construct the graph's edge costs to minimize an engergy E = data + lambda_smooth * smoothness:
        // Labels:
        //   source s (0) = outside
        //   sink t (1) = inside
        // Data term (TODO: tune these bogus params):
        //   P(constraint in x | x = outside) = 1
        //   P(no constraint in x | x = outside) = 0
        //   P(constraint in x | x = inside) = 0
        //   P(no constraint in x | x = inside) = 1
        const double P_constr_X0 = 1.0;
        const double P_no_constr_X0 = 0.0;
        const double P_constr_X1 = 0.0;
        const double P_no_constr_X1 = 1.0;

        //const double lambda_smooth = 0.3; //0.75;  // Good values approx. < 0.5 to 1
        const double lambda_smooth = 0.05; //0.75;  // Good values approx. < 0.5 to 1

        // Construct the graph's data terms
        for (it = dt.finite_cells_begin(); it != dt.finite_cells_end(); it++) {
            int node = mapCellHandleToIndex.find(it)->second;

            double tetVolume = fabs(dt.tetrahedron(it).volume());

            if (it->info().getVoteCount()) {
                // node X has constraints
                graph.addTWeights(node, P_constr_X0 * tetVolume, P_constr_X1 * tetVolume);
            }
            else {
                // node X has no constraint
                graph.addTWeights(node, P_no_constr_X0 * tetVolume, P_no_constr_X1 * tetVolume);
            }
        }

        // Iterate over finite facets to construct the graph's regularization
        for (Delaunay3::Finite_facets_iterator it = dt.finite_facets_begin(); it != dt.finite_facets_end(); it++) {
            // If the facet contains a bounding vert, it won't be added to the isosurface, so don't penalize it with a smoothness cost.
            bool bContainsBoundsVert = false;
            for (int i = 0; i < 4; i++) {
                if (i == it->second) continue;
                if (std::find(vecBoundsHandles.begin(), vecBoundsHandles.end(), it->first->vertex(i)) != vecBoundsHandles.end()) {
                    bContainsBoundsVert = true;  break;
                }
            }
            if (! bContainsBoundsVert) {
                double smoothness_cost = lambda_smooth * sqrt(dt.triangle(*it).squared_area());
                graph.addEdge(mapCellHandleToIndex[it->first], mapCellHandleToIndex[it->first->neighbor(it->second)], smoothness_cost, smoothness_cost);
            }
        }

        // Run the maxflow algorithm to determine the labeling
        graph.maxflow();
        //double flow = graph.maxflow();
        //cerr << "Max Flow: " << flow << endl;

        // Iterate over finite facets to extract the mesh's triangles from the graph's mincut labeling
        for (Delaunay3::Finite_facets_iterator itFacet = dt.finite_facets_begin(); itFacet != dt.finite_facets_end(); itFacet++) {
            // If one adjacent cell is empty, and the other is not, and if the facet contains no vertex from the bounding vertices,
            // then add a triangle to the mesh (w/ correct orientation).
            bool bFacetCellKept = ! graph.whatSegment(mapCellHandleToIndex[itFacet->first]);
            bool bMirrorCellKept = ! graph.whatSegment(mapCellHandleToIndex[itFacet->first->neighbor(itFacet->second)]);
            if (bFacetCellKept && ! bMirrorCellKept) {
                bool bContainsBoundsVert = false;
                for (int i = 0; i < 4; i++) {
                    if (i == itFacet->second) continue;
                    if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) { bContainsBoundsVert = true;  break; }
                }
                if (! bContainsBoundsVert) {
                    Delaunay3::Facet fTmp = dt.mirror_facet(*itFacet); // The normal points inward so mirror the facet
                    Matrix tmpTri(1, 3);
                    vector<Delaunay3::Vertex_handle> vecTri;

                    facetToTri(fTmp, vecTri);
                    tmpTri(0) = hmapVertexHandleToIndex[vecTri[0]];
                    tmpTri(1) = hmapVertexHandleToIndex[vecTri[1]];
                    tmpTri(2) = hmapVertexHandleToIndex[vecTri[2]];
                    tris.push_back(tmpTri);
                }
            }
            else if (bMirrorCellKept && ! bFacetCellKept) {
                bool bContainsBoundsVert = false;
                for (int i = 0; i < 4; i++) {
                    if (i == itFacet->second) continue;
                    if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) { bContainsBoundsVert = true;  break; }
                }
                if (! bContainsBoundsVert) {
                    Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
                    Matrix tmpTri(1, 3);
                    vector<Delaunay3::Vertex_handle> vecTri;

                    facetToTri(fTmp, vecTri);
                    tmpTri(0) = hmapVertexHandleToIndex[vecTri[0]];
                    tmpTri(1) = hmapVertexHandleToIndex[vecTri[1]];
                    tmpTri(2) = hmapVertexHandleToIndex[vecTri[2]];
                    tris.push_back(tmpTri);
                }
            }
        }
    }
}

#endif