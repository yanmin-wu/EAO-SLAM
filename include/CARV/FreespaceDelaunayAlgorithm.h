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


#ifndef __FREESPACEDELAUNAYALGORITHM_H
#define __FREESPACEDELAUNAYALGORITHM_H

#include <vector>
#include <string>
#include <set>
#include <map>
#include <utility>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include "CARV/lovimath.h"
#include "CARV/Matrix.h"
#include "CARV/GraphWrapper_Boost.h"

// CGAL-related includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/intersections.h>

using namespace std;

// Set the algorithm's forgetting heuristic parameter "K" here, and whether it's used.
//#define NO_HEURISTIC_K
#define HEURISTIC_K 1
//#define HEURISTIC_K 5

namespace dlovi {
    class FreespaceDelaunayAlgorithm;
    class FreespaceDelaunayAlgorithm {
    public:

        // CGAL-related typedefs
        typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
        typedef K::Point_3 Point;
        typedef K::Triangle_3 Triangle;
        typedef K::Segment_3 Segment;

        // Our inner class for holding custom info for each cell in a 3D Delaunay tetrahedrization
        class Delaunay3CellInfo {
        public:
            // Sorted-Set related struct
            struct FSConstraint;
            struct LtConstraint {
                bool operator()(const pair<int, int> x, const pair<int, int> y) const { if (x.first < y.first) return true; else return (x.first == y.first) && (x.second < y.second); }
            };
            struct LtFSConstraint {
                bool operator()(const FSConstraint x, const FSConstraint y) const { if (x.first < y.first) return true; else return (x.first == y.first) && (x.second < y.second); }
            };
            struct FSConstraint {
                FSConstraint() { first = -1; second = -1; fNearestNeighborDist = numeric_limits<float>::infinity(); }
                FSConstraint(int camIndex, int featureIndex) { first = camIndex; second = featureIndex; fNearestNeighborDist = numeric_limits<float>::infinity(); }
                FSConstraint(const FSConstraint & ref) { first = ref.first; second = ref.second; pNearestNeighbor = ref.pNearestNeighbor; fNearestNeighborDist = ref.fNearestNeighborDist; }
                FSConstraint & operator=(const FSConstraint & ref) { if (&ref != this){ first = ref.first; second = ref.second; pNearestNeighbor = ref.pNearestNeighbor; fNearestNeighborDist = ref.fNearestNeighborDist; } return *this; }
                operator std::pair<int, int>() const { return std::make_pair(first, second); }

                int first;
                int second;
                mutable set<FSConstraint, LtFSConstraint>::iterator pNearestNeighbor;
                mutable float fNearestNeighborDist;

                // Trick c++ "const"-ness so that we can change the nearest neighbor information in the set from an iterator:
                void setNearestNeighbor(const set<FSConstraint, LtFSConstraint>::iterator itNearest, const float dist) const {
                    pNearestNeighbor = itNearest;
                    fNearestNeighborDist = dist;
                }
                void resetNearestNeighborDist() const { fNearestNeighborDist = numeric_limits<float>::infinity(); }
            };

            // Constructors (it must be default-constructable)
            Delaunay3CellInfo() { m_voteCount = 0; m_bNew = true; m_nMaxConstraintsKept = HEURISTIC_K; }
            Delaunay3CellInfo(const Delaunay3CellInfo & ref) { setVoteCount(ref.getVoteCount()); setIntersections(ref.getIntersections()); if (!ref.isNew()) markOld(); m_nMaxConstraintsKept = ref.m_nMaxConstraintsKept; }

            // Getters
            int getVoteCount() const { return m_voteCount; }
            const set<FSConstraint, LtFSConstraint> & getIntersections() const { return m_setIntersections; }
            bool isNew() const { return m_bNew; }

            // Setters
            void setVoteCount(const int voteCount) { m_voteCount = voteCount; }
            void setIntersections(const set<FSConstraint, LtFSConstraint> & ref) { m_setIntersections = ref; }

            // Public Methods
#ifdef NO_HEURISTIC_K
            void incrementVoteCount() { m_voteCount++; }
#else
            void incrementVoteCount() { if (m_voteCount < m_nMaxConstraintsKept) m_voteCount++; }
#endif
            void decrementVoteCount() { if (m_voteCount > 0) m_voteCount--; }
            bool isKeptByVoteCount(const int nVoteThresh = 1) const { if (getVoteCount() < nVoteThresh) return true;  return false; }
#ifdef NO_HEURISTIC_K
            template <class T>
        void addIntersection(int camIndex, int featureIndex, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
          m_setIntersections.insert(m_setIntersections.end(), FSConstraint(camIndex, featureIndex));
        }
#else
            template <class T>
            void addIntersection(int camIndex, int featureIndex, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
                FSConstraint incoming(camIndex, featureIndex);
                if ((int)m_setIntersections.size() < m_nMaxConstraintsKept) {
                    // The constraint set is not full, so insert the incoming free-space constraint and update nearest neighbor info.
                    set<FSConstraint, LtFSConstraint>::iterator it, itIncoming;
                    itIncoming = m_setIntersections.insert(m_setIntersections.end(), incoming);
                    for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
                        if (it == itIncoming) continue;
                        // Asymmetric metric:
                        float curDist = distFSConstraint(*itIncoming, *it, vecVertexHandles, vecCamCenters);
                        float curDist2 = distFSConstraint(*it, *itIncoming, vecVertexHandles, vecCamCenters);
                        // Update incoming
                        if (curDist < itIncoming->fNearestNeighborDist)
                            itIncoming->setNearestNeighbor(it, curDist);
                        // Update *it:
                        if (curDist2 < it->fNearestNeighborDist)
                            it->setNearestNeighbor(itIncoming, curDist2);
                    }
                }
                else{
                    // The constraint set is full, so apply the spatial cover heuristic to determine whether or not to insert the incoming free-space constraint

                    // Quick return / rejection on the case that m_nMaxConstraintsKept == 1.
                    if (m_nMaxConstraintsKept == 1) return;

                    float minDist = numeric_limits<float>::infinity();
                    set<FSConstraint, LtFSConstraint>::iterator it, it2, itEject;
                    for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
                        float curDist = distFSConstraint(incoming, *it, vecVertexHandles, vecCamCenters);
                        if (curDist < it->fNearestNeighborDist)
                            break; // REJECT
                        float curDist2 = distFSConstraint(*it, incoming, vecVertexHandles, vecCamCenters);
                        if (curDist2 < it->fNearestNeighborDist)
                            break; // REJECT
                        // Update incoming
                        if (curDist < incoming.fNearestNeighborDist)
                            incoming.setNearestNeighbor(it, curDist);
                        // Update minDist & itEject
                        if (it->fNearestNeighborDist < minDist) {
                            minDist = it->fNearestNeighborDist;
                            itEject = it;
                        }
                    }

                    if (it == m_setIntersections.end()) {
                        // No rejection, so insert incoming and evict itEject.

                        // For an asymmetric metric, incoming might have its nearest neighbor ejected.  If so, compute the 2nd nearest neighbor
                        if (incoming.pNearestNeighbor == itEject) {
                            incoming.fNearestNeighborDist = numeric_limits<float>::infinity();
                            for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
                                if (it2 == itEject) continue;
                                float curDist = distFSConstraint(incoming, *it2, vecVertexHandles, vecCamCenters);
                                if (curDist < incoming.fNearestNeighborDist)
                                    incoming.setNearestNeighbor(it2, curDist);
                            }
                        }

                        // Recompute nearest neighbors that previously pointed to itEject.
                        for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
                            if (it2->pNearestNeighbor == itEject) { // implicity "continue;"'s if it2 == itEject
                                // Recompute the nearest neighbor for it2:
                                it2->resetNearestNeighborDist();
                                for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
                                    if (it == itEject || it == it2) continue;
                                    float curDist = distFSConstraint(*it2, *it, vecVertexHandles, vecCamCenters);
                                    if (curDist < it2->fNearestNeighborDist)
                                        it2->setNearestNeighbor(it, curDist);
                                }
                            }
                        }

                        // Finally erase itEject and insert incoming
                        m_setIntersections.erase(itEject);
                        m_setIntersections.insert(m_setIntersections.end(), incoming);
                    }
                }
            }
#endif
#ifdef NO_HEURISTIC_K
            template <class T>
        void removeIntersection(int camIndex, int featureIndex, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
          m_setIntersections.erase(FSConstraint(camIndex, featureIndex));
      }
#else
            template <class T>
            void removeIntersection(int camIndex, int featureIndex, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
                if ((int)m_setIntersections.size() <= 1) {
                    // No nearest neighbor info needs to be updated
                    m_setIntersections.erase(FSConstraint(camIndex, featureIndex));
                }
                else {
                    // The nearest neighbor info needs to be updated
                    set<FSConstraint, LtFSConstraint>::iterator it, it2, itEject;

                    itEject = m_setIntersections.find(FSConstraint(camIndex, featureIndex));
                    if (itEject == m_setIntersections.end())
                        return; // wasn't in the set to begin with

                    for (it = m_setIntersections.begin(); it != m_setIntersections.end(); it++) {
                        if (it == itEject) continue;
                        if (it->pNearestNeighbor == itEject) {
                            // Then recompute the nearest neighbor for it:
                            it->resetNearestNeighborDist();
                            for (it2 = m_setIntersections.begin(); it2 != m_setIntersections.end(); it2++) {
                                if (it2 == itEject || it2 == it) continue;
                                float curDist = distFSConstraint(*it, *it2, vecVertexHandles, vecCamCenters);
                                if (curDist < it->fNearestNeighborDist)
                                    it->setNearestNeighbor(it2, curDist);
                            }
                        }
                    }

                    // Finally, erase it.
                    m_setIntersections.erase(itEject);
                }
            }
#endif
            template <class T>
            float distFSConstraint(const FSConstraint & x, const FSConstraint & y, const vector<T> & vecVertexHandles, const vector<Matrix> & vecCamCenters) {
                return distFSConstraintTriangleAreaAaron(x, y, vecVertexHandles, vecCamCenters);
            }
            void clearIntersections() { m_setIntersections.clear(); }
            void markOld() { m_bNew = false; }

            // Operators (It must be assignable)
            Delaunay3CellInfo & operator=(const Delaunay3CellInfo & rhs)
            { if (this != & rhs) { setVoteCount(rhs.getVoteCount()); setIntersections(rhs.getIntersections()); if (! rhs.isNew()) markOld(); } return *this; }

        private:
            // Private Methods
            template <class T>
            float distFSConstraintTriangleAreaAaron(const FSConstraint & x, const FSConstraint & y, const vector<T> & vecVertexHandles,
                                                    const vector<Matrix> & vecCamCenters) {
                // Asymmetric distance heuristic.
                // Sum of two triangle areas, use the base segment PQ as constraint x, and the two points from y as R1 and R2.
                // Note: For efficiency, to avoid unnecessary division by 2 and square-roots, use the sum of twice-the-areas squared = squared area of parallelograms.
                const Matrix & P = vecCamCenters[x.first];
                Matrix Q(3, 1); Q(0) = vecVertexHandles[x.second]->point().x(); Q(1) = vecVertexHandles[x.second]->point().y(); Q(2) = vecVertexHandles[x.second]->point().z();
                const Matrix & R1 = vecCamCenters[y.first];
                Matrix R2(3, 1); R2(0) = vecVertexHandles[y.second]->point().x(); R2(1) = vecVertexHandles[y.second]->point().y(); R2(2) = vecVertexHandles[y.second]->point().z();

                // Vector distances
                Matrix PQ(Q - P);
                Matrix PR1(R1 - P);
                Matrix PR2(R2 - P);

                // Sum of squared areas of parallelograms
                Matrix PQxPR1(PQ.cross(PR1));
                Matrix PQxPR2(PQ.cross(PR2));
                return PQxPR1.dot(PQxPR1) + PQxPR2.dot(PQxPR2);
            }

            // Private Members
            int m_nMaxConstraintsKept;
            int m_voteCount;
            set<FSConstraint, LtFSConstraint> m_setIntersections;
            bool m_bNew;
        };

        // CGAL-related typedefs for Delaunay triangulation, 3-D
        typedef CGAL::Triangulation_vertex_base_3<K> Vb;
        typedef CGAL::Triangulation_hierarchy_vertex_base_3<Vb> Vbh;
        typedef CGAL::Triangulation_cell_base_with_info_3<Delaunay3CellInfo, K> Cb;
        typedef CGAL::Triangulation_data_structure_3<Vbh, Cb> Tds;
        typedef CGAL::Delaunay_triangulation_3<K, Tds> Dt;
        typedef CGAL::Triangulation_hierarchy_3<Dt> Delaunay3;
        typedef Delaunay3::Point PointD3;

        // Graph-cuts related typedefs
        // typedef Graph<double, double, double> Graph_t; // Boykov & Kolmogorov's Code: TODO: implement a GraphWrapper for this.
        typedef GraphWrapper_Boost Graph_t; // Boykov & Kolmogorov's Code

        // Hashing-related structs:
        struct HashVertHandle{
            size_t operator()(const Delaunay3::Vertex_handle x) const{ return (size_t)(&(*x)); } // use pointer to create hash
        };
        struct EqVertHandle{
            bool operator()(const Delaunay3::Vertex_handle x, const Delaunay3::Vertex_handle y) const{ return x == y; }
        };

        // Constructors
        FreespaceDelaunayAlgorithm();
        FreespaceDelaunayAlgorithm(const vector<Matrix> & points, const vector<Matrix> & cams, const vector<Matrix> & camCenters,
                                   const vector<Matrix> & principleRays, const vector<vector<int> > & visibilityList);
        FreespaceDelaunayAlgorithm(const vector<Matrix> & points, const vector<Matrix> & cams, const vector<Matrix> & camCenters,
                                   const vector<Matrix> & principleRays, const vector<Matrix> & normals);
        FreespaceDelaunayAlgorithm(const FreespaceDelaunayAlgorithm & ref);

        // Getters
        const vector<Matrix> & getPoints() const;
        Matrix getPoint(const int index) const;
        int numPoints() const;
        const vector<Matrix> & getCams() const;
        Matrix getCam(const int index) const;
        const vector<Matrix> & getCamCenters() const;
        Matrix getCamCenter(const int index) const;
        const vector<Matrix> & getPrincipleRays() const;
        Matrix getPrincipleRay(const int index) const;
        const vector<vector<int> > & getVisibilityList() const;
        const vector<int> & getVisibilityList(const int index) const;
        int numCams() const;
        double getBoundsMin() const;
        double getBoundsMax() const;

        // Setters
        void setPoints(const vector<Matrix> & ref);
        void setCams(const vector<Matrix> & ref);
        void setCamCenters(const vector<Matrix> & ref);
        void setPrincipleRays(const vector<Matrix> & ref);
        void setVisibilityList(const vector<vector<int> > & ref);

        void addPoint(const Matrix & ref);
        void addCamCenter(const Matrix & ref);
        void addVisibilityPair(const int camIndex, const int pointIndex);
        void addVisibilityPair(const std::pair<int, int> visibilityPair);

        // Operators
        FreespaceDelaunayAlgorithm & operator=(const FreespaceDelaunayAlgorithm & rhs);

        // Public Methods
        bool isVisible(const int pointIndex, const int viewIndex) const;
        void generateVisibilityFromNormals(const vector<Matrix> & normals, const double nFrontFacingAngleThreshold = pi / 3.0, const double nFov = pi / 2.0);

        void TetrahedronBatchMethod(Delaunay3 & dt, const bool bUseViewingOrder = false) const;
        void IterateTetrahedronMethod(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int frameIndex) const;
        void removeVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int pointIndex) const;
        void removeVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const set<int> & setPointIndices) const; // for batch deletes
        void moveVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int pointIndex) const;
        void moveVertex(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const vector<int> & arrPointIndices) const;
        void applyConstraint(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int camIndex, const int pointIndex) const;
        void removeConstraint(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, const int camIndex, const int pointIndex) const;

        void tetsToTris(const Delaunay3 & dt, vector<Matrix> & points, list<Matrix> & tris, const int nVoteThresh = 1) const;
        int writeObj(const string filename, const vector<Matrix> & points, const list<Matrix> & tris) const;
        void writeObj(ostream & outfile, const vector<Matrix> & points, const list<Matrix> & tris) const;

        void calculateBoundsValues(); // TODO: Refactor.  E.g. move back to private, declare friend classes that need access, e.g. SFMTranscriptInterface_Delaunay

    private:
        // Private Methods
        void copy(const vector<Matrix> & points, const vector<Matrix> & cams, const vector<Matrix> & camCenters,
                  const vector<Matrix> & principleRays, const vector<vector<int> > & visibilityList);
        void createBounds(Delaunay3 & dt) const;
        void markTetrahedraCrossingConstraint(Delaunay3 & dt, const Delaunay3::Vertex_handle hndlQ, const Segment & constraint) const;
        void markTetrahedraCrossingConstraintWithBookKeeping(Delaunay3 & dt, const vector<Delaunay3::Vertex_handle> & vecVertexHandles, const Delaunay3::Vertex_handle hndlQ,
                                                             const Segment & constraint, const int camIndex, const int featureIndex, const bool bOnlyMarkNew = false) const;
        void addNewlyObservedFeatures(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles, vector<int> & localVisList, const vector<int> & originalLocalVisList) const;
        void addNewlyObservedFeature(Delaunay3 & dt, vector<Delaunay3::Vertex_handle> & vecVertexHandles,
                                     set<pair<int, int>, Delaunay3CellInfo::LtConstraint> & setUnionedConstraints, const PointD3 & Q, const int nPointIndex) const;
        bool triangleConstraintIntersectionTest(const Delaunay3::Facet & tri, const Matrix & segSrc, const Matrix & segDest) const;
        bool triangleConstraintIntersectionTest(bool & bCrossesInteriorOfConstraint, const vector<Matrix> & points, const Matrix & tri, const pair<Matrix, Matrix> & constraint) const;
        bool cellTraversalExitTest(int & f, const Delaunay3::Cell_handle tetCur, const Delaunay3::Cell_handle tetPrev, const Matrix & matQ, const Matrix & matO) const;
        void facetToTri(const Delaunay3::Facet & f, vector<Delaunay3::Vertex_handle> & vecTri) const;
        double timestamp() const;
        void tetsToTris_naive(const Delaunay3 & dt, vector<Matrix> & points, list<Matrix> & tris, const int nVoteThresh) const;
        void tetsToTris_maxFlowSimple(const Delaunay3 & dt, vector<Matrix> & points, list<Matrix> & tris, const int nVoteThresh) const;

        // Private Members
        vector<Matrix> m_points;
        vector<Matrix> m_cams;
        vector<Matrix> m_camCenters;
        vector<Matrix> m_principleRays;
        vector<vector<int> > m_visibilityList;
        double m_nBoundsMin;
        double m_nBoundsMax;
        mutable map<int, int> m_mapPoint_VertexHandle; // TODO: Refactor
    };
}

#endif