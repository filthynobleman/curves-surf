/**
 * @file        hamiltonian.hpp
 * 
 * @brief       An interface for finding Hamiltonian paths in graphs.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-18
 */
#pragma once


#include <crs/graph.hpp>


namespace crs
{


bool FindHamiltonianPath(const crs::Graph& G,
                         crs::GraphPath& Path,
                         size_t NumThreads = 8);
void FindHamiltonianPathForced(const crs::Graph& G,
                               crs::GraphPath& Path,
                               const std::vector<std::vector<double>>& Dists,
                               size_t NumThreads = 8);

crs::Graph ForceDiracProperty(const crs::Graph& G,
                              const std::vector<std::vector<double>>& Dists);


} // namespace crs
