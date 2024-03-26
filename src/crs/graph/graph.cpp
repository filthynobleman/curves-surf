/**
 * @file        graph.cpp
 * 
 * @brief       Implements crs::Graph.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-07
 */
#include <crs/graph.hpp>
#include <crs/hash.hpp>

#include <sstream>
#include <exception>
#include <cassert>
#include <algorithm>
#include <random>
#include <map>
#include <queue>



crs::Graph::Graph(size_t NVerts)
{
    m_Edges.clear();
    m_Weights.clear();
    if (NVerts == 0)
        return;

    m_Edges.resize(NVerts);
    m_Weights.resize(NVerts);
}

crs::Graph::Graph(const std::vector<std::vector<double>>& Distances)
    : crs::Graph(Distances.size())
{
    for (size_t i = 0; i < NumVertices(); ++i)
    {
        for (size_t j = i + 1; j < NumVertices(); ++j)
            AddEdge(i, j, Distances[i][j]);
    }
}

crs::Graph::Graph(const crs::Graph& G)
{
    m_Edges = G.m_Edges;
    m_Weights = G.m_Weights;
}
crs::Graph& crs::Graph::operator=(const crs::Graph& G)
{
    m_Edges = G.m_Edges;
    m_Weights = G.m_Weights;
    return *this;
}

crs::Graph::Graph(crs::Graph&& G)
{
    m_Edges = std::move(G.m_Edges);
    m_Weights = std::move(G.m_Weights);
}
crs::Graph& crs::Graph::operator=(crs::Graph&& G)
{
    m_Edges = std::move(G.m_Edges);
    m_Weights = std::move(G.m_Weights);
    return *this;
}

crs::Graph::~Graph() { }




size_t crs::Graph::NumVertices() const      { return m_Edges.size(); }
size_t crs::Graph::NumEdges() const         { return m_NEdges; }
size_t crs::Graph::NumAdjacents(size_t VID) const
{
    assert(VID < NumVertices());
    return m_Edges[VID].size();
}


std::pair<size_t, double> crs::Graph::GetAdjacent(size_t VID, size_t i) const
{
    assert(VID < NumVertices());
    assert(i < NumAdjacents(VID));
    return { m_Edges[VID][i], m_Weights[VID][i] };
}

crs::Edge crs::Graph::GetAdjacentEdge(size_t VID, size_t i) const
{
    auto Adj = GetAdjacent(VID, i);
    return crs::Edge(VID, Adj.first, Adj.second);
}

bool crs::Graph::GetEdge(size_t V1, size_t V2, crs::Edge& E) const
{
    assert(V1 < NumVertices());
    assert(V2 < NumVertices());
    for (size_t i = 0; i < m_Edges[V1].size(); ++i)
    {
        if (m_Edges[V1][i] == V2)
        {
            E = crs::Edge(V1, V2, m_Weights[V1][i]);
            return true;
        }
    }

    return false;
}



void crs::Graph::AddVertex() { AddVertices(1); }
void crs::Graph::AddVertices(size_t NVerts)
{
    if (NVerts == 0)
        return;
    
    size_t TotVerts = NumVertices() + NVerts;
    m_Edges.resize(TotVerts);
    m_Weights.resize(TotVerts);
}

void crs::Graph::RemoveVertex(size_t VID)
{
    assert(VID < NumVertices());
    m_NEdges -= m_Edges[VID].size();
    m_Edges.erase(m_Edges.begin() + VID);
    m_Weights.erase(m_Weights.begin() + VID);
    for (size_t i = 0; i < m_Edges.size(); ++i)
    {
        for (size_t j = 0; j < m_Edges[i].size(); ++j)
        {
            if (m_Edges[i][j] == VID)
            {
                m_Edges[i].erase(m_Edges[i].begin() + j);
                m_Weights[i].erase(m_Weights[i].begin() + j);
                break;
            }
        }
        
        for (size_t j = 0; j < m_Edges[i].size(); ++j)
        {
            if (m_Edges[i][j] > VID)
                m_Edges[i][j]--;
        }
    }
}



void crs::Graph::AddEdge(size_t V1, size_t V2, double W)
{
    assert(V1 < NumVertices());
    assert(V2 < NumVertices());
    bool Found = false;
    for (size_t i = 0; i < m_Edges[V1].size(); ++i)
    {
        if (m_Edges[V1][i] == V2)
        {
            m_Weights[V1][i] = W;
            Found = true;
        }
    }
    if (Found)
    {
        for (size_t i = 0; i < m_Edges[V2].size(); ++i)
        {
            if (m_Edges[V2][i] == V1)
                m_Weights[V2][i] = W;
        }

        return;
    }

    m_Edges[V1].emplace_back(V2);
    m_Edges[V2].emplace_back(V1);
    m_Weights[V1].emplace_back(W);
    m_Weights[V2].emplace_back(W);

    m_NEdges++;
}

void crs::Graph::AddEdge(const crs::Edge& E)
{
    AddEdge(E.FirstVertex(), E.SecondVertex(), E.Weight());
}

void crs::Graph::RemoveEdge(size_t V1, size_t V2)
{
    assert(V1 < NumVertices());
    assert(V2 < NumVertices());
    bool Found = false;
    for (size_t i = 0; i < m_Edges[V1].size(); ++i)
    {
        if (m_Edges[V1][i] == V2)
        {
            m_Edges[V1].erase(m_Edges[V1].begin() + i);
            m_Weights[V1].erase(m_Weights[V1].begin() + i);
            Found = true;
            break;
        }
    }

    if (!Found)
        return;

    for (size_t i = 0; i < m_Edges[V2].size(); ++i)
    {
        if (m_Edges[V2][i] == V1)
        {
            m_Edges[V2].erase(m_Edges[V2].begin() + i);
            m_Weights[V2].erase(m_Weights[V2].begin() + i);
            Found = true;
            break;
        }
    }

    m_NEdges--;
}








void crs::Graph::SortAdjacents(crs::AdjacencyOrdering Ordering,
                               bool Ascending)
{
    std::mt19937 Eng(0);
    std::uniform_real_distribution<float> Distr(0.0f, 1.0f);


    

    for (size_t i = 0; i < NumVertices(); ++i)
    {
        if (Ordering == crs::AdjacencyOrdering::RANDOM)
        {
            std::vector<std::tuple<float, size_t, double>> Adjs;
            Adjs.reserve(NumAdjacents(i));
            for (size_t j = 0; j < Adjs.size(); ++j)
                Adjs.emplace_back(Distr(Eng), m_Edges[i][j], m_Weights[i][j]);
            if (Ascending)
                std::sort(Adjs.begin(), Adjs.end());
            else
                std::sort(Adjs.begin(), Adjs.end(), std::greater<std::tuple<float, size_t, double>>{});
            for (size_t j = 0; j < Adjs.size(); ++j)
            {
                m_Edges[i][j] = std::get<1>(Adjs[j]);
                m_Weights[i][j] = std::get<2>(Adjs[j]);
            }
        }
        else if (Ordering == crs::AdjacencyOrdering::INDEX)
        {
            std::vector<std::tuple<size_t, double>> Adjs;
            Adjs.reserve(NumAdjacents(i));
            for (size_t j = 0; j < Adjs.size(); ++j)
                Adjs.emplace_back(m_Edges[i][j], m_Weights[i][j]);
            if (Ascending)
                std::sort(Adjs.begin(), Adjs.end());
            else
                std::sort(Adjs.begin(), Adjs.end(), std::greater<std::tuple<size_t, double>>{});
            for (size_t j = 0; j < Adjs.size(); ++j)
            {
                m_Edges[i][j] = std::get<0>(Adjs[j]);
                m_Weights[i][j] = std::get<1>(Adjs[j]);
            }
        }
        else if (Ordering == crs::AdjacencyOrdering::DISTANCE)
        {
            std::vector<std::tuple<double, size_t>> Adjs;
            Adjs.reserve(NumAdjacents(i));
            for (size_t j = 0; j < Adjs.size(); ++j)
                Adjs.emplace_back( m_Weights[i][j], m_Edges[i][j]);
            if (Ascending)
                std::sort(Adjs.begin(), Adjs.end());
            else
                std::sort(Adjs.begin(), Adjs.end(), std::greater<std::tuple<double, size_t>>{});
            for (size_t j = 0; j < Adjs.size(); ++j)
            {
                m_Edges[i][j] = std::get<1>(Adjs[j]);
                m_Weights[i][j] = std::get<0>(Adjs[j]);
            }
        }
        else if (Ordering == crs::AdjacencyOrdering::DEGREE)
        {
            std::vector<std::tuple<size_t, double, size_t>> Adjs;
            Adjs.reserve(NumAdjacents(i));
            for (size_t j = 0; j < Adjs.size(); ++j)
                Adjs.emplace_back(NumAdjacents(m_Edges[i][j]), m_Weights[i][j], m_Edges[i][j]);
            if (Ascending)
                std::sort(Adjs.begin(), Adjs.end());
            else
                std::sort(Adjs.begin(), Adjs.end(), std::greater<std::tuple<size_t, double, size_t>>{});
            for (size_t j = 0; j < Adjs.size(); ++j)
            {
                m_Edges[i][j] = std::get<2>(Adjs[j]);
                m_Weights[i][j] = std::get<1>(Adjs[j]);
            }
        }
        else
            throw std::runtime_error("Invalid adjacency ordering type.");
    }
}


size_t crs::Graph::ConnectedComponents(std::vector<crs::Graph>& CCs,
                                       std::vector<std::vector<size_t>>& Idxs) const
{
    CCs.clear();
    Idxs.clear();
    if (NumVertices() == 0)
        return 0;

    // Find the connected components assigning indices.
    // We can safely use -1 even with size_t, because the underflow guarantees
    // that we will never have that many components.
    std::vector<size_t> CCI;
    CCI.resize(NumVertices(), -1);
    size_t NumCCs = 0;

    // Initialize the stack for a DFS visit.
    std::vector<size_t> Stack;
    Stack.reserve(NumVertices());
    for (size_t Start = 0; Start < NumVertices(); ++Start)
    {
        // If we already visited the starting node, we skip it.
        if (CCI[Start] != -1)
            continue;
        // Otherwise, we start a visit from it.
        Stack.push_back(Start);
        while (!Stack.empty())
        {
            size_t Cur = Stack.back();
            Stack.pop_back();

            // If never visited, it's a new component.
            if (CCI[Cur] == -1)
                CCI[Cur] = NumCCs++;
            
            size_t Deg = NumAdjacents(Cur);
            for (size_t i = 0; i < Deg; ++i)
            {
                size_t Adj = GetAdjacent(Cur, i).first;
                // If visited, skip.
                if (CCI[Adj] != -1)
                    continue;
                // Otherwise, it has the same component as the current vertex
                // and we continue the visit
                CCI[Adj] = CCI[Cur];
                Stack.push_back(Adj);
            }
        }
    }


    // Create the components without the edges
    CCs.resize(NumCCs);
    Idxs.resize(NumCCs);
    // We also keep track of the inverse map of Idxs
    std::vector<size_t> LocIdx;
    LocIdx.resize(NumVertices());
    for (size_t i = 0; i < NumVertices(); ++i)
    {
        CCs[CCI[i]].AddVertex();
        Idxs[CCI[i]].push_back(i);
        LocIdx[i] = Idxs[CCI[i]].size() - 1;
    }

    // We now add the edges
    for (size_t i = 0; i < NumVertices(); ++i)
    {
        // Which component?
        crs::Graph& CC = CCs[CCI[i]];

        size_t Deg = NumAdjacents(i);
        for (size_t jj = 0; jj < Deg; ++jj)
        {
            size_t j;
            double w;
            std::tie(j, w) = GetAdjacent(i, jj);

            // By definition of component, adjacent vertices belong to the same component.
            // We use the inverse mapping for determining which vertices must be joined.
            CC.AddEdge(LocIdx[i], LocIdx[j], w);
        }
    }

    // Return the number of components
    return NumCCs;
}


crs::Graph crs::Graph::MinimalConnected(const std::vector<std::vector<double>>& Dists) const
{
    std::vector<crs::Graph> CCs;
    std::vector<std::vector<size_t>> Idxs;
    size_t NumCCs = ConnectedComponents(CCs, Idxs);
    crs::Graph Connected(*this);
    if (NumCCs == 1)
        return Connected;

    // Create the graph of the connected components by finding
    // the shortest edge among each pair of components
    std::vector<std::vector<crs::Edge>> CCEdges;
    CCEdges.resize(NumCCs);
    crs::Graph CCGraph(NumCCs);
    for (size_t cci = 0; cci < NumCCs; ++cci)
    {
        CCEdges[cci].resize(NumCCs, crs::Edge(-1, -1, std::numeric_limits<double>::infinity()));
        // Matrix transpose
        for (size_t ccj = 0; ccj < cci; ++ccj)
            CCEdges[cci][ccj] = CCEdges[ccj][cci];
        for (size_t ccj = cci + 1; ccj < NumCCs; ++ccj)
        {
            // Find smallest edge cost among these components
            for (size_t i : Idxs[cci])
            {
                for (size_t j : Idxs[ccj])
                {
                    if (Dists[i][j] < CCEdges[cci][ccj].Weight())
                        CCEdges[cci][ccj] = crs::Edge(i, j, Dists[i][j]);
                }
            }
            CCGraph.AddEdge(cci, ccj, CCEdges[cci][ccj].Weight());
        }
    }

    // Find the minimum spanning tree of the connected components graph
    CCGraph = CCGraph.MinimumSpanningTree();
    // Use the edges to connect the original graph
    for (size_t i = 0; i < CCGraph.NumVertices(); ++i)
    {
        for (size_t jj = 0; jj < CCGraph.NumAdjacents(i); ++jj)
        {
            size_t j = CCGraph.GetAdjacent(i, jj).first;
            Connected.AddEdge(CCEdges[i][j]);
        }
    }

    return Connected;
}




crs::Graph crs::Graph::MinimumSpanningTree() const
{
    std::vector<double> Costs;
    Costs.resize(NumVertices(), std::numeric_limits<double>::infinity());
    std::vector<size_t> Parents;
    Parents.resize(NumVertices(), -1);

    std::priority_queue<std::pair<double, size_t>,
                        std::vector<std::pair<double, size_t>>,
                        std::greater<std::pair<double, size_t>>> Q;
    for (size_t i = 0; i < NumVertices(); ++i)
        Q.emplace(std::numeric_limits<double>::infinity(), i);

    std::vector<bool> Forest;
    Forest.resize(NumVertices(), false);

    size_t v;
    double w;
    while (!Q.empty())
    {
        std::tie(w, v) = Q.top();
        Q.pop();

        if (Forest[v])
            continue;

        Forest[v] = true;

        size_t Deg = NumAdjacents(v);
        for (size_t i = 0; i < Deg; ++i)
        {
            size_t k;
            double d;
            std::tie(k, d) = GetAdjacent(v, i);
            if (Forest[k])
                continue;
            if (d >= Costs[k])
                continue;
            Costs[k] = d;
            Parents[k] = v;
            Q.emplace(d, k);
        }
    }

    crs::Graph MST(NumVertices());
    for (size_t i = 0; i < NumVertices(); ++i)
    {
        if (Parents[i] == -1)
            continue;
        MST.AddEdge(i, Parents[i], Costs[i]);
    }

    return MST;
}


bool crs::Graph::IsChainTree(crs::GraphPath& P) const
{
    P.Length = 0.0;
    P.Vertices.clear();
    P.Vertices.reserve(NumVertices());

    std::vector<size_t> Stack;
    for (size_t i = 0; i < NumVertices(); ++i)
    {
        if (NumAdjacents(i) == 1)
        {
            Stack.push_back(i);
            break;
        }
    }
    if (Stack.empty())
        return false;
    
    std::vector<bool> Visited;
    Visited.resize(NumVertices(), false);
    while (!Stack.empty())
    {
        size_t V = Stack.back();
        Stack.pop_back();

        // Since the graph is a chain and the visit starts from an end vertex,
        // then it is not possible to put on stack more than one vertex at time.
        // This means that as we remove a vertex from the stack, the stack must be empty
        if (!Stack.empty())
            return false;

        Visited[V] = true;
        P.Vertices.push_back(V);

        size_t Deg = NumAdjacents(V);
        for (size_t i = 0; i < Deg; ++i)
        {
            size_t W = GetAdjacent(V, i).first;
            if (Visited[W])
                continue;
            Stack.push_back(W);
        }
    }

    return true;
}