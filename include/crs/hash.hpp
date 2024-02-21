/**
 * @file        hash.hpp
 * 
 * @brief       All the std::hash structures for using the classes of this library with
 *              unordered sets and maps.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-07
 */
#pragma once

#include <functional>

#include <crs/graph.hpp>


namespace std
{
    
template<>
struct std::hash<crs::Edge>
{
    std::size_t operator()(const crs::Edge& E) const noexcept
    {
        return E.Hash();
    }
};
    
} // namespace std
