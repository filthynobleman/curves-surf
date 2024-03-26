/**
 * @file        graph.hpp
 * 
 * @brief       A class representing a weighted undirected graph.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-07
 */
#pragma once

#include <vector>
#include <array>

namespace crs
{
    
struct GraphPath
{
    std::vector<size_t> Vertices;
    double Length;
};

class Edge
{
private:
    // The vertices of the edge
    std::array<size_t, 2> m_V;

    // The weight of the edge
    double m_W;

public:
    Edge(size_t v0, size_t v1, double w = 1.0);
    Edge(const crs::Edge& E);
    crs::Edge& operator=(const crs::Edge& E);
    ~Edge();

    // Get the first vertex of the edge (i.e., the smallest ID).
    size_t FirstVertex() const;
    // Get the second vertex of the edge (i.e., the largest ID).
    size_t SecondVertex() const;
    // Get the vertex of the edge that is not VID.
    // If VID is not a vertex of the edge, an exception is thrown.
    size_t OtherVertex(size_t VID) const;
    
    // The weight of the edge.
    double Weight() const;

    // Hash function for edges.
    size_t Hash() const;

    friend bool operator==(const crs::Edge& E1, const crs::Edge& E2);
    friend bool operator!=(const crs::Edge& E1, const crs::Edge& E2);
    friend bool operator<=(const crs::Edge& E1, const crs::Edge& E2);
    friend bool operator<(const crs::Edge& E1, const crs::Edge& E2);
    friend bool operator>=(const crs::Edge& E1, const crs::Edge& E2);
    friend bool operator>(const crs::Edge& E1, const crs::Edge& E2);
};
bool operator==(const crs::Edge& E1, const crs::Edge& E2);
bool operator!=(const crs::Edge& E1, const crs::Edge& E2);
bool operator<=(const crs::Edge& E1, const crs::Edge& E2);
bool operator<(const crs::Edge& E1, const crs::Edge& E2);
bool operator>=(const crs::Edge& E1, const crs::Edge& E2);
bool operator>(const crs::Edge& E1, const crs::Edge& E2);



enum AdjacencyOrdering
{
    // Random ordering
    RANDOM,
    // Order by index
    INDEX,
    // Order by degree
    DEGREE,
    // Order by distance
    DISTANCE
};


    
class Graph
{
private:
    // The adjacency list
    std::vector<std::vector<size_t>> m_Edges;

    // The weights of the edges
    std::vector<std::vector<double>> m_Weights;

    // Total number of edges
    size_t m_NEdges;
    
public:
    // Initialize a graph with the given number of nodes and no edges.
    Graph(size_t NVerts = 0);
    // Initializes a clique graph from the given set of distances.
    Graph(const std::vector<std::vector<double>>& Distances);
    // Copy the given graph.
    Graph(const crs::Graph& G);
    Graph(crs::Graph&& G);
    crs::Graph& operator=(const crs::Graph& G);
    crs::Graph& operator=(crs::Graph&& G);
    ~Graph();


    // Number of vertices
    size_t NumVertices() const;
    // Total number of edges.
    size_t NumEdges() const;
    // Number of edges incident on vertex VID.
    size_t NumAdjacents(size_t VID) const;
    
    // The i-th adjacent vertex to vertex VID, and the length of the edge.
    // Edges are ordered by increasing length.
    std::pair<size_t, double> GetAdjacent(size_t VID, size_t i) const;
    // The i-th edge incident to vertex VID. Edges are ordered by increasing length.
    crs::Edge GetAdjacentEdge(size_t VID, size_t i) const;

    // Get the edge between V1 and V2, if it exists. Otherwise, the content of E is not valid.
    // The return value determines if the edge does exists.
    bool GetEdge(size_t V1, size_t V2, crs::Edge& E) const;


    // Add a new vertex with ID NumVertices()
    void AddVertex();
    // Add NVerts vertices, with sequential IDs, starting from NumVertices()
    void AddVertices(size_t NVerts);
    // Remove the vertex VID. This will decrement all the following IDs by one,
    // potentially invalidating all the previously saved identifiers.
    void RemoveVertex(size_t VID);

    // Add the edge between V1 and V2, assigning it weight W.
    // If the edge already exists, this method updates its weight.
    void AddEdge(size_t V1, size_t V2, double W);
    // Add the edge E. If the edge already exists, this method updates
    // its weight using the value E.Weight().
    void AddEdge(const crs::Edge& E);
    // Remove the edge between V1 and V2. If the edge does not exist, this method does nothing.
    void RemoveEdge(size_t V1, size_t V2);

    // Sort the adjacency list of each vertex using the specified criterion.
    // Sorting can be ascending or descending.
    void SortAdjacents(crs::AdjacencyOrdering Ordering = crs::INDEX,
                       bool Ascending = true);


    // Separate the graph into its connected components.
    // Vector CCs will contain the graphs representing the single components.
    // Idxs[i][j] contains the index of the j-th vertex of the i-th component in this graph.
    // The method returns the number of connected components.
    size_t ConnectedComponents(std::vector<crs::Graph>& CCs,
                               std::vector<std::vector<size_t>>& Idxs) const;

    // Connects all the (eventually) separated components by chosing the edges of minimal
    // cost among the input weighted connectivity.
    crs::Graph MinimalConnected(const std::vector<std::vector<double>>& Dists) const;

    // Computes the minimum spanning tree of the graph
    crs::Graph MinimumSpanningTree() const;

    // Determines whether this graph is a chain tree.
    // If the returning value is true, the parameter P is set to the visit order of the chain.
    // Otherwise, the content of P is invalid.
    bool IsChainTree(crs::GraphPath& P) const;
};

} // namespace crs
