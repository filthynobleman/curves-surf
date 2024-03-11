/**
 * @file        hamiltonian.cpp
 * 
 * @brief       Implements Hamiltonian paths.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-18
 */
#include <crs/hamiltonian.hpp>
#include <thread>
#include <random>


size_t _NumAvailableNeighs(const crs::Graph& G,
                           const std::vector<bool>& Visited,
                           size_t VID,
                           size_t StartVertex)
{
    // Preconditions: 
    //  - Visited[StartVertex] = true
    //  - Visited[CurVertex] = false

    size_t Deg = G.NumAdjacents(VID);
    size_t Count = 0;
    for (size_t ii = 0; ii < Deg; ++ii)
    {
        // Every unvisited neighbor is available, as well as the starting
        // vertex and the current vertex
        size_t i = G.GetAdjacent(VID, ii).first;
        if (i == StartVertex)
            Count++;
        else if (!Visited[i])
            Count++;
    }

    return Count;
}

bool _Prune(const crs::Graph& G,
            const std::vector<bool>& Visited,
            size_t StartVertex,
            size_t CurVertex)
{
    // Preconditions: 
    //  - Visited[StartVertex] = true
    //  - Visited[CurVertex] = false

    for (size_t i = 0; i < G.NumVertices(); ++i)
    {
        // We don't care about visited vertices, except for the starting vertex
        if (Visited[i] && i != StartVertex)
            continue;

        // How many neighbors are available for a visit?
        size_t NumAN = _NumAvailableNeighs(G, Visited, i, StartVertex);

        // If no neighbor is available for an unvisited vertex, we must prune
        if (NumAN == 0)
            return true;

        // If only one neighbor is available for some vertex, besides the current
        // vertex and the starting vertex, we must prune
        if (i != CurVertex && i != StartVertex && NumAN == 1)
            return true;
    }

    return false;
}


void FindHamiltonianPathST(const crs::Graph& G,
                           crs::GraphPath& Path,
                           size_t StartVertex,
                           size_t& Found)
{
    // Determines whether a vertex is visited or not
    std::vector<bool> Visited;
    Visited.resize(G.NumVertices(), false);

    // The recurrence stack
    std::vector<std::tuple<size_t, double, bool>> Stack;
    Stack.reserve(G.NumVertices() * G.NumVertices() * 2);
    Stack.emplace_back(StartVertex, 0.0, true);

    // Initialize the path to an empty one
    Path.Vertices.clear();
    Path.Length = 0.0;
    Path.Vertices.reserve(G.NumVertices());


    size_t CurVertex;
    double CurLen;
    bool FirstExtraction;
    size_t Deg;
    while (!Stack.empty())
    {
        std::tie(CurVertex, CurLen, FirstExtraction) = Stack.back();
        Stack.pop_back();

        // This means we came back from a visit in the sub-tree, so there is nothing more
        // we can do with this node now.
        if (!FirstExtraction)
        {
            Visited[CurVertex] = false;
            Path.Vertices.pop_back();
            continue;
        }

        // This means we have visited all the vertices, so we must check if we 
        // have found a cycle
        if (Path.Vertices.size() == G.NumVertices() - 1)
        {
            crs::Edge E(CurVertex, StartVertex);
            // If the cycle is there, we can return the path
            if (G.GetEdge(CurVertex, StartVertex, E))
            {
                Path.Vertices.push_back(CurVertex);
                Path.Length = CurLen + E.Weight();
                Found = true;
                return;
            }
            // Otherwise, we must roll back
            continue;
        }

        // This means this sub-tree do not offer solutions, so we can prune
        if (_Prune(G, Visited, StartVertex, CurVertex))
            continue;


        // Mark as visited and get ready for the second extraction after the
        // entire sub-tree has been visited
        Stack.emplace_back(CurVertex, CurLen, false);
        Visited[CurVertex] = true;
        Path.Vertices.push_back(CurVertex);

        // Push the unvisited neighbors on the stack
        Deg = G.NumAdjacents(CurVertex);
        for (size_t jj = 0; jj < Deg; ++jj)
        {
            size_t j;
            double w;
            std::tie(j, w) = G.GetAdjacent(CurVertex, jj);

            if (Visited[j])
                continue;
            
            Stack.emplace_back(j, CurLen + w, true);
        }

    };
    
    // If we get here, no path has been found
    Found = false;
}

bool crs::FindHamiltonianPath(const crs::Graph& G,
                              crs::GraphPath& Path,
                              size_t NumThreads)
{
    std::vector<std::thread> Threads;
    std::vector<crs::GraphPath> Paths;
    std::vector<size_t> Founds;
    Threads.reserve(NumThreads);
    Paths.resize(NumThreads);
    Founds.resize(NumThreads, false);

    std::mt19937 Eng(0);
    std::uniform_int_distribution<size_t> Distr(0, G.NumVertices() - 1);

    for (size_t i = 0; i < NumThreads; ++i)
    {
        Threads.emplace_back(FindHamiltonianPathST,
                             std::ref(G),
                             std::ref(Paths[i]),
                             Distr(Eng),
                             std::ref(Founds[i]));
    }
    for (size_t i = 0; i < NumThreads; ++i)
        Threads[i].join();

    Path = Paths[0];
    bool Found = Founds[0];
    for (size_t i = 1; i < NumThreads; ++i)
    {
        if (Founds[i] == 0)
            continue;

        if (!Found)
            Path = Paths[i];
        else if (Paths[i].Length < Path.Length)
            Path = Paths[i];

        Found = true;
    }
    return Found;
}

void crs::FindHamiltonianPathForced(const crs::Graph& G,
                                    crs::GraphPath& Path,
                                    const std::vector<std::vector<double>>& Dists,
                                    size_t NumThreads)
{
    if (crs::FindHamiltonianPath(G, Path, NumThreads))
        return;

    crs::Graph GF = crs::ForceDiracProperty(G, Dists);
    crs::FindHamiltonianPath(GF, Path, NumThreads);
}


#include <iostream>

crs::Graph crs::ForceDiracProperty(const crs::Graph& G,
                                   const std::vector<std::vector<double>>& Dists)
{
    crs::Graph GH(G);

    // Vector of degree/vertex pairs
    std::vector<std::pair<size_t, size_t>> VDeg;
    VDeg.resize(GH.NumVertices());
    for (size_t i = 0; i < GH.NumVertices(); ++i)
        VDeg[i] = { GH.NumAdjacents(i), i };

    // Sort by descending degree
    std::sort(VDeg.begin(), VDeg.end(), std::greater<std::pair<size_t, size_t>>{});

    // Start filling from the higher degree, but adding low degree first
    crs::Edge E(0, 1);
    for (size_t i = 0; i < GH.NumVertices(); ++i)
    {
        size_t ii = VDeg[i].second;
        bool HasChanged = VDeg[i].first < GH.NumVertices() / 2;
        for (size_t j = GH.NumVertices() - 1; VDeg[i].first < GH.NumVertices() / 2; --j)
        {
            size_t jj = VDeg[j].second;
            if (ii == jj)
                continue;
            // If edge is there, we have nothing to do
            if (G.GetEdge(ii, jj, E))
                continue;

            // Otherwise, we add the edge
            GH.AddEdge(ii, jj, Dists[ii][jj]);

            VDeg[i].first++;
            VDeg[j].first++;
        }

        // We can safely sort the vector, because at every iteration the value of
        // the elements increases. After the i-th iteration, the first i elements
        // have at least value N/2. Since we sort in descending order, we do not
        // care what happen to the array, because we know that there are at least
        // i elements with at least value N/2. So, when we move to i + 1, we know
        // that on the left everything is fine and we can safely ignore it.
        if (HasChanged)
            std::sort(VDeg.begin(), VDeg.end(), std::greater<std::pair<size_t, size_t>>{});
    }

    return GH;
}