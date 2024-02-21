/**
 * @file        sig.cpp
 * 
 * @brief       Implements the spheres of influence.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-18
 */
#include <crs/sig.hpp>



void crs::SpheresOfInfluenceInplace(crs::Graph& G)
{
    std::vector<double> NN;
    NN.resize(G.NumVertices(), std::numeric_limits<double>::infinity());

    for (size_t i = 0; i < G.NumVertices(); ++i)
    {
        size_t Deg = G.NumAdjacents(i);
        for (size_t jj = 0; jj < Deg; ++jj)
        {
            size_t j;
            double w;
            std::tie(j, w) = G.GetAdjacent(i, jj);
            if (j <= i)
                continue;
            
            NN[i] = std::min(NN[i], w);
            NN[j] = std::min(NN[j], w);
        }
    }

    std::vector<std::pair<size_t, size_t>> ERemove;
    for (size_t i = 0; i < G.NumVertices(); ++i)
    {
        size_t Deg = G.NumAdjacents(i);
        for (size_t jj = 0; jj < Deg; ++jj)
        {
            size_t j;
            double w;
            std::tie(j, w) = G.GetAdjacent(i, jj);
            if (j <= i)
                continue;
            if (NN[i] + NN[j] >= w)
                continue;
            
            ERemove.emplace_back(i, j);
        }
    }

    for (auto e : ERemove)
    {
        G.RemoveEdge(e.first, e.second);
    }
}


crs::Graph crs::SpheresOfInfluence(const crs::Graph& G)
{
    std::vector<double> NN;
    NN.resize(G.NumVertices(), std::numeric_limits<double>::infinity());


    for (size_t i = 0; i < G.NumVertices(); ++i)
    {
        size_t Deg = G.NumAdjacents(i);
        for (size_t jj = 0; jj < Deg; ++jj)
        {
            size_t j;
            double w;
            std::tie(j, w) = G.GetAdjacent(i, jj);
            if (j <= i)
                continue;
            
            NN[i] = std::min(NN[i], w);
            NN[j] = std::min(NN[j], w);
        }
    }

    crs::Graph SIG(NN.size());
    
    for (size_t i = 0; i < G.NumVertices(); ++i)
    {
        size_t Deg = G.NumAdjacents(i);
        for (size_t jj = 0; jj < Deg; ++jj)
        {
            size_t j;
            double w;
            std::tie(j, w) = G.GetAdjacent(i, jj);
            if (j <= i)
                continue;
            if (NN[i] + NN[j] < w)
                continue;

            SIG.AddEdge(i, j, w);
        }
    }

    return SIG;
}