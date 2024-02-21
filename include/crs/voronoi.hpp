/**
 * @file        voronoi.hpp
 * 
 * @brief       Class defining the Voronoi partitioning of a triangle mesh.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-07
 */
#pragma once

#include <geometrycentral/surface/surface_mesh.h>
#include <geometrycentral/surface/intrinsic_geometry_interface.h>
#include <geometrycentral/surface/heat_method_distance.h>

#include <crs/graph.hpp>
#include <map>


namespace crs
{


enum DistanceBackend
{
    // Dijkstra's algorithm
    DIJKSTRA,
    // Exact geodesics
    EXACT,
    // Heat method
    HEAT
};

    
class VoronoiPartitioning
{
private:
    geometrycentral::surface::SurfaceMesh& m_Mesh;
    geometrycentral::surface::IntrinsicGeometryInterface& m_Geometry;

    crs::DistanceBackend m_DBackend;
    geometrycentral::surface::HeatMethodDistanceSolver* m_HSolver;

    std::vector<size_t> m_Samples;
    std::map<size_t, size_t> m_SampleMap;
    std::vector<size_t> m_Partitioning;
    std::vector<double> m_Distances;

    std::vector<std::vector<double>> m_SSDist;


public:
    VoronoiPartitioning(geometrycentral::surface::SurfaceMesh& Mesh,
                        geometrycentral::surface::IntrinsicGeometryInterface& Geometry,
                        crs::DistanceBackend DistBackend = crs::DistanceBackend::DIJKSTRA);
    ~VoronoiPartitioning();

    // The number of samples inducing the partitioning.
    size_t NumSamples() const;
    // Determines if VID is the index of a sample.
    bool IsSample(size_t VID) const;
    // Determine if V is a samples vertex.
    bool IsSample(const geometrycentral::surface::Vertex& V) const;
    // Get the i-th sample, with i < NumSamples().
    size_t GetSample(size_t i) const;
    // Add the mesh's vertex with index VID to the sample list and updates the partitioning.
    // VID must be a valid vertex index.
    void AddSample(size_t VID);
    // Add the mesh's vertex V to the sample list and updates the partitioning.
    // V must be a vertex of the mesh.
    void AddSample(const geometrycentral::surface::Vertex& V);

    // Get partition of vertex VID.
    // VID must be a valid vertex index, and the partitioning must contain at least one sample.
    size_t GetPartition(size_t VID) const;
    // Get partition of given vertex.
    // V must be a vertex of the mesh, and the partitioning must contain at least one sample.
    size_t GetPartition(const geometrycentral::surface::Vertex& V) const;

    // Get the entire Voronoi partitioning of the mesh.
    // The partitioning must contian at least one sample.
    const std::vector<size_t>& GetPartitioning() const;


    // Build the dual Voronoi connectivity.
    // Partitioning must contain at least one sample.
    crs::Graph DualVoronoi() const;
};

} // namespace crs
