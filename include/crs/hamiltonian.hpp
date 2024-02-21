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
    
struct GraphPath
{
    std::vector<size_t> Vertices;
    double Length;
};


bool FindHamiltonianPath(const crs::Graph& G,
                         crs::GraphPath& Path,
                         size_t NumThreads = 8);

crs::GraphPath ForceHamiltonianPath(const crs::Graph& G,
                                    size_t NumThreads = 8);


} // namespace crs
