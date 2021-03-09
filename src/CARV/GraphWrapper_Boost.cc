#ifndef __GRAPHWRAPPER_BOOST_CPP
#define __GRAPHWRAPPER_BOOST_CPP

#include "CARV/GraphWrapper_Boost.h"

namespace dlovi {

  // Constructors and Destructors

  GraphWrapper_Boost::GraphWrapper_Boost() {
    m_capacity = get(edge_capacity, m_g);
    m_color = get(vertex_color, m_g);
    m_rev = get(edge_reverse, m_g);
  }
  GraphWrapper_Boost::GraphWrapper_Boost(int nVertices, int nEdges) : m_g(nVertices) {
    m_capacity = get(edge_capacity, m_g);
    m_color = get(vertex_color, m_g);
    m_rev = get(edge_reverse, m_g);
  }
  GraphWrapper_Boost::~GraphWrapper_Boost() {}

  // Getters

  bool GraphWrapper_Boost::whatSegment(int node) const {
    return m_color[node] != m_color[m_t];
  }
  int GraphWrapper_Boost::getSource() const {
    return m_s;
  }
  int GraphWrapper_Boost::getSink() const {
    return m_t;
  }

  // Public Methods

  void GraphWrapper_Boost::addNodes(int numVertices) {
    for(int i = 0; i < numVertices; ++i)
      add_vertex(m_g);
  }
  void GraphWrapper_Boost::addSource() {
    m_s = add_vertex(m_g);
  }
  void GraphWrapper_Boost::addSink() {
    m_t = add_vertex(m_g);
  }
  void GraphWrapper_Boost::addTWeights(int node, double sourceWeight, double sinkWeight) {
    addEdge(m_s, node, sourceWeight);
    addEdge(node, m_t, sinkWeight);
  }
  void GraphWrapper_Boost::addEdge(int node1, int node2, double weight, double revWeight) {
    Traits::edge_descriptor edge;
    Traits::edge_descriptor revEdge;
    bool bExists;

    tie(edge, bExists) = boost::edge(node1, node2, m_g);
    if (! bExists) {
      edge = add_edge(node1, node2, m_g).first;
      revEdge = add_edge(node2, node1, m_g).first;
      m_rev[edge] = revEdge;
      m_rev[revEdge] = edge;
    }
    else
      revEdge = m_rev[edge];

    m_capacity[edge] += weight;
    m_capacity[revEdge] += revWeight;
  }
  double GraphWrapper_Boost::maxflow() {
    return boykov_kolmogorov_max_flow(m_g, m_s, m_t);
  }
  void GraphWrapper_Boost::print() const {
    graph_traits<Graph>::vertex_iterator u_iter, u_end;
    graph_traits<Graph>::out_edge_iterator ei, e_end;

    cout << "NumVertices = " << num_vertices(m_g) << endl;

    for (tie(u_iter, u_end) = vertices(m_g); u_iter != u_end; ++u_iter) {
      cout << "Vertex: " << *u_iter << "    cut: " << (whatSegment(*u_iter) ? "source" : "sink") << endl;
      //cout << "Vertex: " << *u_iter << "    color: " << color[*u_iter] << endl;
      for (tie(ei, e_end) = out_edges(*u_iter, m_g); ei != e_end; ++ei) {
        cout << "Edge: (" << *u_iter << ", " << target(*ei, m_g) << ")" << "    capacity: " << m_capacity[*ei] << "    rev: (" << source(m_rev[*ei], m_g) << ", " << target(m_rev[*ei], m_g) << ")" << endl;
      }
    }
  }
}

#endif
