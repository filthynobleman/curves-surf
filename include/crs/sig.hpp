/**
 * @file        sig.hpp
 * 
 * @brief       A routine computing the Spheres of Influence sub-graph of a given graph.
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
    
crs::Graph SpheresOfInfluence(const crs::Graph& G);
void SpheresOfInfluenceInplace(crs::Graph& G);

} // namespace crs
