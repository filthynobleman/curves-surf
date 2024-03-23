#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"
#include <geometrycentral/surface/exact_geodesics.h>
#include <geometrycentral/surface/heat_method_distance.h>
#include <geometrycentral/surface/fast_marching_method.h>
#include <geometrycentral/surface/flip_geodesics.h>
#include <geometrycentral/surface/mesh_graph_algorithms.h>

#include <crs/graph.hpp>
#include <crs/voronoi.hpp>
#include <crs/io.hpp>
#include <crs/sig.hpp>
#include <crs/hamiltonian.hpp>
#include <crs/settings.hpp>

#include <array>
#include <chrono>

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

// Some algorithm parameters
std::vector<size_t> samples;
crs::Settings settings;
size_t NThreads = 32;

std::chrono::system_clock::time_point TStart;
std::chrono::system_clock::time_point TEnd;

void StartTimer()
{
    TStart = std::chrono::system_clock::now();
}

double StopTimer()
{
    TEnd = std::chrono::system_clock::now();
    auto ETA = TEnd - TStart;
    auto ETAus = std::chrono::duration_cast<std::chrono::microseconds>(ETA).count();
    return ETAus * 1.0e-6;
}


void doWork() {
    StartTimer();
    // Compute Voronoi partitioning of sample points
    crs::VoronoiPartitioning VP(*mesh, *geometry, settings.DistanceFunction);
    for (int i = 0; i < samples.size(); ++i)
        VP.AddSample(samples[i]);

    // Create dual Voronoi graph
    crs::Graph VPG = VP.DualVoronoi();

    // If needed, compute intersection with SIG
    if (settings.SIGSubGraph)
        crs::SpheresOfInfluenceInplace(VPG);
    // Graph construction time
    double ETA = StopTimer();

    // Output graph and points for visualization
    std::vector<std::vector<Vector3>> Paths;
    crs::ExportPoints(settings.OutputPrefix + "vertices.obj", samples, *geometry);
    crs::PathsFromGraph(VPG, *mesh, *geometry, samples, Paths);
    if (settings.SIGSubGraph)
        crs::ExportEdgeNetwork(settings.OutputPrefix + "sigdt.obj", Paths);
    else
        crs::ExportEdgeNetwork(settings.OutputPrefix + "voronoi.obj", Paths);

    crs::Graph MST(VP.GetSampleSampleDistances());
    MST = MST.MinimumSpanningTree();
    crs::GraphPath MSTP;
    if (MST.IsChainTree(MSTP))
    {
        std::cout << "Graph has chain MST." << std::endl;
        crs::PathsFromGraphPath(MST, MSTP, *mesh, *geometry, samples, Paths);
        crs::ExportEdgeNetwork(settings.OutputPrefix + "mstgraph.obj", Paths);
    }
    else
    {
        crs::PathsFromGraph(MST, *mesh, *geometry, samples, Paths);
        crs::ExportEdgeNetwork(settings.OutputPrefix + "mstgraph.obj", Paths);
    }

    // Compute Hamiltonian cycle
    if (settings.MultipleComponents)
    {
        StartTimer();
        // In case of multiple components, we separate the graph into multiple subgraphs
        std::vector<crs::Graph> CCs;
        std::vector<std::vector<size_t>> Idxs;
        size_t NumCCs = 0;
        NumCCs = VPG.ConnectedComponents(CCs, Idxs);
        ETA += StopTimer();

        // Compute Hamiltonian cycle for each component
        double TotLength = 0.0;
        for (size_t i = 0; i < NumCCs; ++i)
        {
            StartTimer();
            // Compute mapping from local graph indices to mesh samples
            std::vector<size_t> LocIdxs = Idxs[i];
            for (size_t j = 0; j < LocIdxs.size(); ++j)
                LocIdxs[j] = samples[LocIdxs[j]];
            
            // Find Hamiltonian cycle
            crs::GraphPath P;
            CCs[i].SortAdjacents(settings.HamiltonianBias);
            bool HPFound = crs::FindHamiltonianPath(CCs[i], P, NThreads);
            std::cout << StopTimer() << std::endl;
            if (!HPFound)
            {
                // If not found, check if we should enforce Dirac's property
                if (!settings.ForceHamiltonianPath)
                {
                    std::cerr << "Cannot find a Hamiltonian cycle." << std::endl;
                    return;
                }
                // Get sample-sample distances among the component's samples
                std::vector<std::vector<double>> SSDists;
                SSDists.resize(Idxs[i].size());
                for (size_t jj = 0; jj < SSDists.size(); ++jj)
                {
                    for (size_t kk = 0; kk < SSDists.size(); ++kk)
                    {
                        SSDists[jj].push_back(VP.GetSampleSampleDistances()[Idxs[i][jj]][Idxs[i][kk]]);
                    }
                }

                // Enforce Dirac's property and recompute Hamiltonian cycle
                CCs[i] = crs::ForceDiracProperty(CCs[i], SSDists);
                CCs[i].SortAdjacents(settings.HamiltonianBias);
                crs::FindHamiltonianPath(CCs[i], P, NThreads);
            }
            ETA += StopTimer();
            TotLength += P.Length;


            // Compute the paths for visualization
            std::vector<std::vector<geometrycentral::Vector3>> PathsLoc;
            crs::PathsFromGraphPath(CCs[i], P, *mesh, *geometry, LocIdxs, PathsLoc);
            if (i == 0)
                Paths = PathsLoc;
            else
                Paths.insert(Paths.end(), PathsLoc.begin(), PathsLoc.end());
        }
        std::cout << "Elapsed time is " << ETA << " seconds." << std::endl;

        // Output run info
        std::ofstream Stream;
        Stream.open(settings.OutputPrefix + "info.csv", std::ios::out);
        if (!Stream.is_open())
        {
            std::cerr << "Cannot open file " << settings.OutputPrefix << "info.csv for writing." << std::endl;
            return;
        }

        Stream << "Mesh,Samples,NumSamples,NumCCs,TotLength,Time\n";
        Stream << settings.InputMesh << ',';
        Stream << settings.InputSamples << ',';
        Stream << samples.size() << ',';
        Stream << NumCCs << ',';
        Stream << TotLength << ',';
        Stream << ETA << '\n';

        Stream.close();
    }
    else
    {
        // Find Hamiltonian path on a single component
        StartTimer();
        crs::GraphPath P;
        VPG.SortAdjacents(settings.HamiltonianBias);
        bool HPFound = crs::FindHamiltonianPath(VPG, P, NThreads);
        if (!HPFound)
        {
            // If not found, check if we should enforce Dirac's property
            if (!settings.ForceHamiltonianPath)
            {
                std::cerr << "Cannot find a Hamiltonian cycle.";
                return;
            }
            // Enforce Dirac's property and recompute Hamiltonian cycle
            VPG = crs::ForceDiracProperty(VPG, VP.GetSampleSampleDistances());
            VPG.SortAdjacents(settings.HamiltonianBias);
            crs::FindHamiltonianPath(VPG, P, NThreads);
        }
        ETA += StopTimer();
        std::cout << "Elapsed time is " << ETA << " seconds." << std::endl;

        crs::PathsFromGraphPath(VPG, P, *mesh, *geometry, samples, Paths);

        // Output run info
        std::ofstream Stream;
        Stream.open(settings.OutputPrefix + "info.csv", std::ios::out);
        if (!Stream.is_open())
        {
            std::cerr << "Cannot open file " << settings.OutputPrefix << "info.csv for writing." << std::endl;
            return;
        }

        Stream << "Mesh,Samples,NumSamples,NumCCs,TotLength,Time\n";
        Stream << settings.InputMesh << ',';
        Stream << settings.InputSamples << ',';
        Stream << samples.size() << ',';
        Stream << 1 << ',';
        Stream << P.Length << ',';
        Stream << ETA << '\n';

        Stream.close();
    }

    crs::ExportEdgeNetwork(settings.OutputPrefix + "cycles.obj", Paths);
}

int main(int argc, char **argv) {

    if (argc == 1)
    {
        std::cerr << "Please provide the settigns file." << std::endl;
        return EXIT_FAILURE;
    }

    settings = crs::LoadSettings(argv[1]);

    crs::ImportPoints(settings.InputSamples, samples);

    // Load mesh
    std::tie(mesh, geometry) = readManifoldSurfaceMesh(settings.InputMesh);

    std::cout << "Mesh loaded" << std::endl;

    // std::mt19937 Eng(0);
    // std::uniform_int_distribution<size_t> Distr(0, mesh->nVertices() - 1);
    // for (size_t i = 0; i < 30; ++i)
    // {
    //     samples.push_back(Distr(Eng));
    //     // std::cout << samples.back() << std::endl;
    // }
    std::sort(samples.begin(), samples.end());
    auto SEnd = std::unique(samples.begin(), samples.end());
    samples.erase(SEnd, samples.end());
    std::cout << samples.size() << std::endl;

    doWork();


    return EXIT_SUCCESS;
}
