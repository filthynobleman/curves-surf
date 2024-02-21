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
                           size_t StartVertex,
                           size_t CurVertex)
{
    size_t Deg = G.NumAdjacents(VID);
    size_t Count = 0;
    for (size_t ii = 0; ii < Deg; ++ii)
    {
        size_t i = G.GetAdjacent(VID, ii).first;
        if (i == StartVertex || i == CurVertex)
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
    for (size_t i = 0; i < G.NumVertices(); ++i)
    {
        if (Visited[i] && i != StartVertex)
            continue;

        if (_NumAvailableNeighs(G, Visited, i, StartVertex, CurVertex) <= 1)
            return true;
    }

    return false;
}


bool _FindHamiltonianPath(const crs::Graph& G,
                          std::vector<bool>& Visited,
                          size_t& NumVisited,
                          size_t CurVertex,
                          size_t StartVertex,
                          crs::GraphPath& Path)
{
    Visited[CurVertex] = true;
    NumVisited++;
    if (NumVisited == G.NumVertices())
    {
        crs::Edge E(CurVertex, StartVertex);
        if (G.GetEdge(CurVertex, StartVertex, E))
        {
            Path.Vertices.push_back(CurVertex);
            Path.Length += E.Weight();
            return true;
        }
        Visited[CurVertex] = false;
        NumVisited--;
        return false;
    }

    if (_Prune(G, Visited, StartVertex, CurVertex))
    {
        Visited[CurVertex] = false;
        NumVisited--;
        return false;
    }
    
    size_t Deg = G.NumAdjacents(CurVertex);
    for (size_t jj = 0; jj < Deg; ++jj)
    {
        size_t j;
        double w;
        std::tie(j, w) = G.GetAdjacent(CurVertex, jj);

        if (Visited[j])
            continue;

        Path.Vertices.push_back(CurVertex);
        double OldLen = Path.Length;
        Path.Length += w;
        if (_FindHamiltonianPath(G, Visited, NumVisited, j, StartVertex, Path))
            return true;
        Path.Vertices.pop_back();
        Path.Length = OldLen;
    }

    Visited[CurVertex] = false;
    NumVisited--;
    return false;
}


void FindHamiltonianPathST(const crs::Graph& G,
                           crs::GraphPath& Path,
                           size_t StartVertex,
                           size_t& Found)
{
    std::vector<bool> Visited;
    Visited.resize(G.NumVertices(), false);
    size_t NumVisited = 0;
    Path = crs::GraphPath{ {}, 0.0 };

    Found = _FindHamiltonianPath(G, Visited, NumVisited, StartVertex, StartVertex, Path) ? 1 : 0;
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
    bool Found = false;
    for (size_t i = 1; i < NumThreads; ++i)
    {
        if (Founds[i] == 0)
            continue;
        Found = true;
        if (Paths[i].Length < Path.Length)
            Path = Paths[i];
    }
    return Found;
}


// void _ForceHamiltonianPathST(const crs::Graph& G,
//                              std::vector<bool>& Visited,
//                              size_t& NumVisited,
//                              size_t CurVertex,
//                              crs::GraphPath& Path,
//                              crs::GraphPath& Longest)
// {
//     Visited[CurVertex] = true;
//     NumVisited++;
//     if (NumVisited == G.NumVertices())
//     {
//         Path.Vertices.push_back(CurVertex);
//         return;
//     }
    
//     size_t Deg = G.NumAdjacents(CurVertex);
//     size_t VCount = 0;
//     for (size_t jj = 0; jj < Deg; ++jj)
//     {
//         size_t j;
//         double w;
//         std::tie(j, w) = G.GetAdjacent(CurVertex, jj);

//         if (Visited[j])
//         {
//             VCount++;
//             continue;
//         }

//         Path.Vertices.push_back(CurVertex);
//         double OldLen = Path.Length;
//         Path.Length += w;
//         if (_FindHamiltonianPath(G, Visited, NumVisited, j, Path))
//             return;
//         Path.Vertices.pop_back();
//         Path.Length = OldLen;
//     }
//     if (VCount == Deg)
//     {

//     }

//     Visited[CurVertex] = false;
//     NumVisited--;
//     return;
// }