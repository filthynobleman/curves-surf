/**
 * @file        tsp.hpp
 * 
 * @brief       Solving the traveling salesman problem.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-03-24
 */
#pragma once


#include <crs/graph.hpp>


namespace crs
{
    
void MetricTSP(const crs::Graph& G,
               const std::vector<std::vector<double>>& Dists,
               crs::GraphPath& P,
               size_t NumThreads = 8);

void TSPOptimize(const std::vector<std::vector<double>>& Dists,
                 crs::GraphPath& P);

} // namespace crs
