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



crs::Graph::Graph(size_t NVerts)
{
    m_Edges.clear();
    m_Weights.clear();
    if (NVerts == 0)
        return;

    m_Edges.resize(NVerts);
    m_Weights.resize(NVerts);
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