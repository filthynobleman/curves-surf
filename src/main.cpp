#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"
#include <geometrycentral/surface/exact_geodesics.h>
#include <geometrycentral/surface/heat_method_distance.h>
#include <geometrycentral/surface/fast_marching_method.h>
#include <geometrycentral/surface/flip_geodesics.h>
#include <geometrycentral/surface/mesh_graph_algorithms.h>

#include "args/args.hxx"

#include <crs/graph.hpp>
#include <crs/voronoi.hpp>
#include <crs/io.hpp>
#include <crs/sig.hpp>
#include <crs/hamiltonian.hpp>

#include <array>
#include <chrono>

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

// Some algorithm parameters
std::vector<size_t> samples;

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

// Example computation function -- this one computes and registers a scalar
// quantity
void doWork() {
  crs::VoronoiPartitioning VP(*mesh, *geometry);
  for (int i = 0; i < samples.size(); ++i)
    VP.AddSample(samples[i]);

  crs::Graph VPG = VP.DualVoronoi();
  std::cout << "Computed dual Voronoi." << std::endl;

  crs::SpheresOfInfluenceInplace(VPG);
  std::cout << "Computed SIGDT." << std::endl;

  FlipEdgeNetwork FENet(*mesh, *geometry, {});
  FENet.supportRewinding = true;
  FENet.posGeom = geometry.get();

  std::vector<size_t> Samples;
  std::vector<std::vector<Vector3>> Paths;

  for (size_t i = 0; i < VPG.NumVertices(); ++i)
  {
    Samples.emplace_back(VP.GetSample(i));
    size_t Deg = VPG.NumAdjacents(i);
    for (size_t jj = 0; jj < Deg; ++jj)
    {
      size_t j = VPG.GetAdjacent(i, jj).first;
      if (j <= i)
        continue;
      
      Vertex vi = mesh->vertex(VP.GetSample(i));
      Vertex vj = mesh->vertex(VP.GetSample(j));

      auto dijpath = shortestEdgePath(*geometry, vi, vj);
      FENet.reinitializePath({ dijpath });
      FENet.iterativeShorten();

      Paths.emplace_back(FENet.getPathPolyline3D().front());

      FENet.rewind();
    }
  }

  crs::ExportPoints("test-vertices.obj", Samples, *geometry);
  crs::ExportEdgeNetwork("test-voronoi.obj", Paths);

  std::cout << "Exported graph." << std::endl;

  StartTimer();
  crs::GraphPath P;
  bool HPFound = crs::FindHamiltonianPath(VPG, P);
  double ETA = StopTimer();
  std::cout << "Elapsed time is " << ETA << " seconds." << std::endl;
  if (!HPFound)
  {
    std::cout << "Cannot find a Hamiltonian path." << std::endl;
    return;
  }

  std::cout << "Found a Hamiltonian path of length " << P.Length << std::endl;
  Paths.clear();
  for (size_t i = 0; i < P.Vertices.size(); ++i)
  {
    size_t j = (i + 1) % P.Vertices.size();

    Vertex vi = mesh->vertex(VP.GetSample(P.Vertices[i]));
    Vertex vj = mesh->vertex(VP.GetSample(P.Vertices[j]));

    auto dijpath = shortestEdgePath(*geometry, vi, vj);
    FENet.reinitializePath({ dijpath });
    FENet.iterativeShorten();

    Paths.emplace_back(FENet.getPathPolyline3D().front());

    FENet.rewind();
  }
  
  crs::ExportEdgeNetwork("test-hamilton.obj", Paths);
}

int main(int argc, char **argv) {

  // Configure the argument parser
  args::ArgumentParser parser("geometry-central & Polyscope example project");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");
  args::Positional<std::string> inputSamples(parser, "samples", "The path to a samples file.");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help &h) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError &e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  // Make sure a mesh name was given
  if (!inputFilename) {
    std::cerr << "Please specify a mesh file as argument" << std::endl;
    return EXIT_FAILURE;
  }

  crs::ImportPoints(args::get(inputSamples), samples);

  // Load mesh
  std::tie(mesh, geometry) = readManifoldSurfaceMesh(args::get(inputFilename));

  // std::mt19937 Eng(0);
  // std::uniform_int_distribution<size_t> Distr(0, mesh->nVertices() - 1);
  // for (size_t i = 0; i < 3000; ++i)
  // {
  //   samples.push_back(Distr(Eng));
  //   // std::cout << samples.back() << std::endl;
  // }
  std::sort(samples.begin(), samples.end());
  auto SEnd = std::unique(samples.begin(), samples.end());
  samples.erase(SEnd, samples.end());

  doWork();
  

  return EXIT_SUCCESS;
}
