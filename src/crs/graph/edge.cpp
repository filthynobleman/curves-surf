/**
 * @file        edge.cpp
 * 
 * @brief       Implements crs::Edge.
 * 
 * @author      Filippo Maggioli\n
 *              (maggioli@di.uniroma1.it, maggioli.filippo@gmail.com)\n
 *              Sapienza, University of Rome - Department of Computer Science
 * 
 * @date        2024-02-07
 */
#include <crs/graph.hpp>
#include <sstream>
#include <exception>
#include <functional>


crs::Edge::Edge(size_t v0, size_t v1, double w)
    : m_V({v0, v1}), m_W(w)
{
    if (m_V[0] > m_V[1])
        std::swap(m_V[0], m_V[1]);
}

crs::Edge::Edge(const crs::Edge& E)
    : m_V(E.m_V), m_W(E.m_W)
{ }

crs::Edge& crs::Edge::operator=(const crs::Edge& E)
{
    m_V = E.m_V;
    m_W = E.m_W;

    return *this;
}

crs::Edge::~Edge() { }


size_t crs::Edge::FirstVertex() const   { return m_V[0]; }
size_t crs::Edge::SecondVertex() const  { return m_V[1]; }
double crs::Edge::Weight() const        { return m_W; }

size_t crs::Edge::OtherVertex(size_t VID) const
{
    if (VID == FirstVertex())
        return SecondVertex();
    else if (VID == SecondVertex())
        return FirstVertex();
    else
    {
        std::stringstream ss;
        ss << "Edge { " << FirstVertex() << ", " << SecondVertex() << "} does ";
        ss << "not contain vertex " << VID << '.';
        throw std::runtime_error(ss.str());
    }
}


size_t crs::Edge::Hash() const
{
    std::hash<size_t> h{};
    return h(FirstVertex()) ^ (h(SecondVertex()) << 1);
}





bool crs::operator==(const crs::Edge& E1, const crs::Edge& E2)
{
    return E1.m_V[0] == E2.m_V[0] && E1.m_V[1] == E2.m_V[1];
}
bool crs::operator!=(const crs::Edge& E1, const crs::Edge& E2)
{
    return E1.m_V[0] != E2.m_V[0] || E1.m_V[1] != E2.m_V[1];
}

bool crs::operator<=(const crs::Edge& E1, const crs::Edge& E2)
{
    if (E1.m_V[0] < E2.m_V[0])
        return true;
    if (E1.m_V[0] > E2.m_V[0])
        return false;
    return E1.m_V[1] <= E2.m_V[1];
}
bool crs::operator<(const crs::Edge& E1, const crs::Edge& E2)
{
    if (E1.m_V[0] < E2.m_V[0])
        return true;
    if (E1.m_V[0] > E2.m_V[0])
        return false;
    return E1.m_V[1] < E2.m_V[1];
}
bool crs::operator>=(const crs::Edge& E1, const crs::Edge& E2)
{
    return E2 <= E1;
}
bool crs::operator>(const crs::Edge& E1, const crs::Edge& E2)
{
    return E2 < E1;
}
