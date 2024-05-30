/**
 * @file        tsp.cpp
 * 
 * @brief       
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-03-24
 */
#include <crs/tsp.hpp>

#include <thread>
#include <random>
#include <algorithm>



void _MetricTSPST(const crs::Graph& MST,
                  const std::vector<std::vector<double>>& Dists,
                  size_t Start,
                  crs::GraphPath& P)
{
    std::vector<size_t> Stack;
    Stack.reserve(MST.NumVertices());
    std::vector<bool> Visited;
    Visited.resize(MST.NumVertices(), false);

    P.Vertices.clear();
    P.Vertices.reserve(MST.NumVertices());
    P.Length = 0.0;

    Stack.emplace_back(Start);
    while (!Stack.empty())
    {
        size_t Cur = Stack.back();
        Stack.pop_back();
        if (Visited[Cur])
            continue;

        Visited[Cur] = true;
        P.Vertices.push_back(Cur);

        size_t Deg = MST.NumAdjacents(Cur);
        for (size_t i = 0; i < Deg; ++i)
        {
            size_t j = MST.GetAdjacent(Cur, i).first;
            Stack.push_back(j);
        }
    }

    P.Length = 0.0;
    for (size_t i = 0; i < MST.NumVertices() - 1; ++i)
        P.Length += Dists[P.Vertices[i]][P.Vertices[i + 1]];
    P.Length += Dists[P.Vertices[0]][P.Vertices.back()];

    crs::TSPOptimize(Dists, P);
}


void crs::MetricTSP(const crs::Graph& G,
                    const std::vector<std::vector<double>>& Dists,
                    crs::GraphPath& P,
                    size_t NumThreads)
{
    crs::Graph MST = G.MinimumSpanningTree();
    MST.SortAdjacents(crs::AdjacencyOrdering::RANDOM);
    NumThreads = std::min(NumThreads, MST.NumVertices());
    std::vector<size_t> Sinks;
    for (size_t i = 0; i < MST.NumVertices(); ++i)
    {
        if (MST.NumAdjacents(i) == 1)
            Sinks.push_back(i);
    }
    std::mt19937 Eng(0);
    std::uniform_int_distribution<size_t> Distr(0, G.NumVertices() - 1);
    while (Sinks.size() < NumThreads)
    {
        size_t j = Distr(Eng);
        if (std::find(Sinks.begin(), Sinks.end(), j) != Sinks.end())
            continue;
        Sinks.push_back(j);
    }
    std::uniform_int_distribution<size_t> Distr2(0, Sinks.size() - 1);
    for (size_t i = 0; i < NumThreads; ++i)
    {
        size_t j = Distr2(Eng);
        std::swap(Sinks[i], Sinks[j]);
    }

    std::vector<std::thread> Threads;
    Threads.reserve(NumThreads);
    std::vector<crs::GraphPath> Paths;
    Paths.resize(NumThreads);


    for (size_t i = 0; i < NumThreads; ++i)
    {
        Threads.emplace_back(_MetricTSPST,
                             std::ref(G),
                             std::ref(Dists),
                             Sinks[i],
                             std::ref(Paths[i]));
    }
    for (size_t i = 0; i < NumThreads; ++i)
        Threads[i].join();

    P = Paths[0];
    for (size_t i = 1; i < NumThreads; ++i)
    {
        if (Paths[i].Length < P.Length)
            P = Paths[i];
    }
}


void crs::TSPOptimize(const std::vector<std::vector<double>>& Dists,
                      crs::GraphPath& P)
{
    
    // Iteratively exchange pairs until no optimization is found
    bool HasInterPaths = false;
    do
    {
        HasInterPaths = false;
        for (size_t i = 0; i < P.Vertices.size() - 1; ++i)
        {
            size_t ie = i + 1;
            for (size_t j = ie + 1; j < P.Vertices.size(); ++j)
            {
                size_t je = j + 1;
                if (je == P.Vertices.size())
                    je = 0;

                // If an optimization is found, swap the connections and ask for another iteration
                double ActualLength = Dists[P.Vertices[i]][P.Vertices[ie]] + Dists[P.Vertices[j]][P.Vertices[je]];
                double MaybeLength = Dists[P.Vertices[i]][P.Vertices[j]] + Dists[P.Vertices[je]][P.Vertices[ie]];
                if (MaybeLength < ActualLength)
                {
                    // To swap the edges, we must reverse the connectivity from ie to j included
                    HasInterPaths = true;
                    std::reverse(P.Vertices.begin() + ie, P.Vertices.begin() + j + 1);
                    break;
                }
            }
            // if (HasInterPaths)
            //     break;
        }
    } while (HasInterPaths);
    
}