/**
 * @file        io.hpp
 * 
 * @brief       Input/output routines.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-18
 */
#pragma once


#include <geometrycentral/surface/surface_mesh.h>
#include <geometrycentral/surface/vertex_position_geometry.h>
#include <geometrycentral/surface/flip_geodesics.h>



namespace crs
{
    
void ImportPoints(const std::string& Filename,
                  std::vector<size_t>& Points);
void ExportPoints(const std::string& Filename,
                  const std::vector<size_t>& Points);
void ExportPoints(const std::string& Filename,
                  const std::vector<size_t>& Points,
                  geometrycentral::surface::VertexPositionGeometry& Geometry);

void ExportEdgeNetwork(const std::string& Filename,
                       std::vector<std::vector<geometrycentral::Vector3>>& Paths);

} // namespace crs
