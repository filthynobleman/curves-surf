/**
 * @file        io.cpp
 * 
 * @brief       Implements I/O routines.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-18
 */
#include <crs/io.hpp>
#include <fstream>
#include <sstream>
#include <exception>
#include <geometrycentral/surface/flip_geodesics.h>
#include <geometrycentral/surface/mesh_graph_algorithms.h>


void crs::ImportPoints(const std::string& Filename,
                       std::vector<size_t>& Points)
{
    std::ifstream Stream;
    Stream.open(Filename, std::ios::in);
    if (!Stream.is_open())
    {
        std::stringstream ss;
        ss << "Cannot open file " << Filename << " for reading.";
        throw std::runtime_error(ss.str());
    }

    std::string Line;
    while (!Stream.eof())
    {
        std::getline(Stream, Line);
        if (Line.empty())
            continue;

        Points.emplace_back(std::stoull(Line));
    }

    Stream.close();
}


void crs::ExportPoints(const std::string& Filename,
                       const std::vector<size_t>& Points)
{
    std::ofstream Stream;
    Stream.open(Filename, std::ios::out);
    if (!Stream.is_open())
    {
        std::stringstream ss;
        ss << "Cannot open file " << Filename << " for writing.";
        throw std::runtime_error(ss.str());
    }

    for (size_t i = 0; i < Points.size(); ++i)
        Stream << Points[i] << '\n';

    Stream.close();
}


void crs::ExportPoints(const std::string& Filename,
                       const std::vector<size_t>& Points,
                       geometrycentral::surface::VertexPositionGeometry& Geometry)
{
    std::ofstream Stream;
    Stream.open(Filename, std::ios::out);
    if (!Stream.is_open())
    {
        std::stringstream ss;
        ss << "Cannot open file " << Filename << " for writing.";
        throw std::runtime_error(ss.str());
    }

    Stream << "o " << Filename.substr(0, Filename.rfind('.')) << '\n';
    for (size_t i = 0; i < Points.size(); ++i)
    {
        geometrycentral::Vector3 v = Geometry.vertexPositions[Points[i]];
        Stream << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
    }

    Stream.close();
}

void crs::ExportEdgeNetwork(const std::string& Filename,
                            std::vector<std::vector<geometrycentral::Vector3>>& Paths)
{
    std::ofstream Out;
    Out.open(Filename, std::ios::out);
    if (!Out.is_open())
    {
        std::stringstream ss;
        ss << "Cannot open file " << Filename << " for writing.";
        throw std::runtime_error(ss.str());
    }

    Out << "o " << Filename.substr(0, Filename.rfind('.')) << '\n';
    for (auto& path : Paths)
    {
        for (auto& v : path)
        Out << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
    }
    size_t First = 0;
    size_t Last = 0;
    for (auto& path : Paths)
    {
        First = Last;
        Last = First + path.size();
        for (size_t i = First + 1; i < Last; ++i)
        Out << "l " << i << ' ' << i + 1 << '\n';
    }

    Out.close();
}



void crs::PathsFromGraph(const crs::Graph& G,
                         geometrycentral::surface::ManifoldSurfaceMesh& Mesh,
                         geometrycentral::surface::VertexPositionGeometry& Geometry,
                         const std::vector<size_t>& Graph2Mesh,
                         std::vector<std::vector<geometrycentral::Vector3>>& Paths)
{
    geometrycentral::surface::FlipEdgeNetwork FENet(Mesh, Geometry, {});
    FENet.supportRewinding = true;
    FENet.posGeom = &Geometry;

    Paths.clear();
    for (size_t i = 0; i < G.NumVertices(); ++i)
    {
        size_t Deg = G.NumAdjacents(i);
        for (size_t jj = 0; jj < Deg; ++jj)
        {
            size_t j = G.GetAdjacent(i, jj).first;
            if (j <= i)
                continue;
            
            geometrycentral::surface::Vertex vi = Mesh.vertex(Graph2Mesh[i]);
            geometrycentral::surface::Vertex vj = Mesh.vertex(Graph2Mesh[j]);

            auto dijpath = geometrycentral::surface::shortestEdgePath(Geometry, vi, vj);
            FENet.reinitializePath({ dijpath });
            FENet.iterativeShorten();

            Paths.emplace_back(FENet.getPathPolyline3D().front());

            FENet.rewind();
        }
    }
}

void crs::PathsFromGraphPath(const crs::Graph& G,
                             const crs::GraphPath& GP,
                             geometrycentral::surface::ManifoldSurfaceMesh& Mesh,
                             geometrycentral::surface::VertexPositionGeometry& Geometry,
                             const std::vector<size_t>& Graph2Mesh,
                             std::vector<std::vector<geometrycentral::Vector3>>& Paths)
{
    geometrycentral::surface::FlipEdgeNetwork FENet(Mesh, Geometry, {});
    FENet.supportRewinding = true;
    FENet.posGeom = &Geometry;

    Paths.clear();
    for (size_t i = 0; i < GP.Vertices.size(); ++i)
    {
        size_t j = (i + 1) % GP.Vertices.size();

        geometrycentral::surface::Vertex vi = Mesh.vertex(Graph2Mesh[GP.Vertices[i]]);
        geometrycentral::surface::Vertex vj = Mesh.vertex(Graph2Mesh[GP.Vertices[j]]);

        auto dijpath = geometrycentral::surface::shortestEdgePath(Geometry, vi, vj);
        FENet.reinitializePath({ dijpath });
        FENet.iterativeShorten();

        Paths.emplace_back(FENet.getPathPolyline3D().front());

        FENet.rewind();
    }
}