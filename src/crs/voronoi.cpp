/**
 * @file        voronoi.cpp
 * 
 * @brief       Implements crs::VoronoiPartitioning.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-07
 */
#include <crs/voronoi.hpp>


#include <geometrycentral/surface/mesh_graph_algorithms.h>
#include <geometrycentral/surface/exact_geodesics.h>
#include <geometrycentral/surface/heat_method_distance.h>
#include <geometrycentral/surface/flip_geodesics.h>


crs::VoronoiPartitioning::VoronoiPartitioning(geometrycentral::surface::SurfaceMesh& Mesh,
                                              geometrycentral::surface::IntrinsicGeometryInterface& Geometry,
                                              crs::DistanceBackend DistBackend)
    : m_Mesh(Mesh), m_Geometry(Geometry), m_DBackend(DistBackend)
{
    m_Samples.clear();
    m_SampleMap.clear();
    m_SSDist.clear();

    m_Partitioning.resize(m_Mesh.nVertices());
    m_Distances.resize(m_Mesh.nVertices(), std::numeric_limits<double>::infinity());

    m_HSolver = nullptr;
    if (m_DBackend == crs::DistanceBackend::HEAT)
        m_HSolver = new geometrycentral::surface::HeatMethodDistanceSolver(m_Geometry);
}

crs::VoronoiPartitioning::~VoronoiPartitioning()
{ 
    if (m_HSolver != nullptr)
        delete m_HSolver;
}


size_t crs::VoronoiPartitioning::NumSamples() const { return m_Samples.size(); }
bool crs::VoronoiPartitioning::IsSample(size_t VID) const { return m_SampleMap.find(VID) != m_SampleMap.end(); }
bool crs::VoronoiPartitioning::IsSample(const geometrycentral::surface::Vertex& V) const { return IsSample(V.getIndex()); }
size_t crs::VoronoiPartitioning::GetSample(size_t i) const
{
    assert(i < NumSamples());
    return m_Samples[i];
}

size_t crs::VoronoiPartitioning::GetPartition(size_t VID) const
{
    assert(NumSamples() > 0);
    assert(VID < m_Mesh.nVertices());
    return m_Partitioning[VID];
}
size_t crs::VoronoiPartitioning::GetPartition(const geometrycentral::surface::Vertex& V) const
{
    return GetPartition(V.getIndex());
}

const std::vector<size_t>& crs::VoronoiPartitioning::GetPartitioning() const
{
    assert(NumSamples() > 0);
    return m_Partitioning;
}

const std::vector<std::vector<double>>& crs::VoronoiPartitioning::GetSampleSampleDistances() const
{
    return m_SSDist;
}

void crs::VoronoiPartitioning::AddSample(size_t VID)
{
    assert(VID < m_Mesh.nVertices());
    assert(!IsSample(VID));

    std::vector<double> NewDists;
    NewDists.resize(m_Mesh.nVertices(), std::numeric_limits<double>::infinity());

    if (m_DBackend == crs::DistanceBackend::DIJKSTRA)
    {
        auto DijkDist = geometrycentral::surface::vertexDijkstraDistanceWithinRadius(m_Geometry, m_Mesh.vertex(VID), std::numeric_limits<double>::infinity());
        for (const auto& p : DijkDist)
            NewDists[p.first.getIndex()] = p.second;
    }
    else if (m_DBackend == crs::DistanceBackend::EXACT)
    {
        auto Exact = geometrycentral::surface::exactGeodesicDistance(m_Mesh, m_Geometry, m_Mesh.vertex(VID));
        for (size_t i = 0; i < m_Mesh.nVertices(); ++i)
            NewDists[i] = Exact[i];
    }
    else if (m_DBackend == crs::DistanceBackend::HEAT)
    {
        auto Heat = m_HSolver->computeDistance(m_Mesh.vertex(VID));
        for (size_t i = 0; i < m_Mesh.nVertices(); ++i)
            NewDists[i] = Heat[i];
    }
    else
        throw std::runtime_error("Invalid distance backend type.");

    for (size_t i = 0; i < m_Mesh.nVertices(); ++i)
    {
        if (NewDists[i] < m_Distances[i])
        {
            m_Distances[i] = NewDists[i];
            m_Partitioning[i] = NumSamples();
        }
    }

    

    m_SSDist.emplace_back();
    m_SSDist.back().resize(NumSamples() + 1, 0.0);
    for (size_t i = 0; i < NumSamples(); ++i)
    {
        m_SSDist[i].emplace_back(NewDists[m_Samples[i]]);
        m_SSDist.back()[i] = NewDists[m_Samples[i]];
    }

    m_SampleMap.emplace(VID, m_SampleMap.size());
    m_Samples.emplace_back(VID);
}

void crs::VoronoiPartitioning::AddSample(const geometrycentral::surface::Vertex& V) { AddSample(V.getIndex()); }


crs::Graph crs::VoronoiPartitioning::DualVoronoi() const
{
    std::map<std::pair<size_t, size_t>, double> Edges;
    for (auto e : m_Mesh.edges())
    {
        std::pair<size_t, size_t> ps;
        ps.first = m_Partitioning[e.firstVertex().getIndex()];
        ps.second = m_Partitioning[e.secondVertex().getIndex()];
        if (ps.first == ps.second)
            continue;
        if (ps.first > ps.second)
            std::swap(ps.first, ps.second);
        if (Edges.find(ps) != Edges.end())
            continue;
        Edges.emplace(ps, m_SSDist[ps.first][ps.second]);
    }

    crs::Graph G(NumSamples());
    for (const auto& p : Edges)
        G.AddEdge(p.first.first, p.first.second, p.second);
    
    return G;
}