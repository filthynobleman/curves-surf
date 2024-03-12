/**
 * @file        settings.hpp
 * 
 * @brief       Data structure for handling settings.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-03-12
 */
#pragma once


#include <crs/voronoi.hpp>
#include <string>


namespace crs
{
    
struct Settings
{
    std::string InputMesh;
    std::string InputSamples;
    std::string OutputPrefix;

    bool SIGSubGraph;
    bool MultipleComponents;
    bool ForceHamiltonianPath;

    crs::DistanceBackend DistanceFunction;
    crs::AdjacencyOrdering HamiltonianBias;
};


// Load settings from file.
crs::Settings LoadSettings(const std::string& Filename);


} // namespace crs
