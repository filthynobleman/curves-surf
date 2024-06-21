# Non-Euclidean Curves
This repository contains the implementation of the algorithm for reconstructing curves in non-Euclidean domains, as described in the paper *Reconstructing Curves from Sparse Samples on Riemannian Manifolds*. 
 - DOI: **TBA**
 - PDF: https://arxiv.org/pdf/2404.09661


## Building instructions
Clone the project 
```
git clone --recursive https://github.com/filthynobleman/curves-surf.git
```

> [!WARNING]
> For Windows users, Geometry Central could raise errors at compile time. To avoid it, change
> `deps/geometry-central/deps/CMakeLists.txt:160` by removing the option `SYMBOLIC`.

Please be sure that all the submodules are updated to the latest version
```
git submodule update --init --recursive --remote
```

Then build with CMake
```
mkdir build
cd build
cmake ..
cmake --build . --config release --parallel
```

## Usage
The building process produces a single executable `bin/CurvesSurf[.exe]`, which accepts a
single argument. The argument is a configuration file for a run, containing the following
attributes  

| Attribute | Domain | Comments | Meaning |
| --------- | ------ | -------- | ------- |
| `mesh` | string | **Mandatory** | The path to a file containing a triangular mesh. |
| `samples` | string | **Mandatory** | The path to a file containing a newline separated list of indices of the mesh. |
| `output_prefix` | string | Optional, by default the basename of the input mesh. | The prefix for all the output files. |
| `use_sig` | boolean | Optional, by default **true**. | Wheter or not to use SIGDT connectivity in favor of the dual Voronoi only. |
| `multi_components` | boolean | Optional, by default **true**. | Whether or not to solve for a curve in every connected component of the graph. |
| `force_hamiltonian` | boolean | Optional, by default **false**. | Whether or not to edit the graph in order to force it in becoming Hamiltonian. |
| `distance` | `{ "dijkstra", "heat", "exact" }` | Optional, by default `"dijsktra"`. | Which distance function is used by the algorithm (Dijkstra's algorithm, geodesics in heat, or exact geodesics). |
| `bias` | `{ "none", "degree", "distance" }` | Optional, by default `"none"`. | Which edge choice must be prioritized in findining Hamiltonian cycles (random, smallest degree first, shortest edge first). |

An example configuration file can be found in `examples/settings.json`.  

The algorithm procudes four output files:
 - `<output_prefix>vertices.obj`, containing the 3D coordinates of each sample;
 - `<output_prefix>voronoi.obj`, containing the exact geodesic lines representing the dual Voronoi connectivity;
  - `<output_prefix>sigdt.obj`, if option `use_sig` is given, the SIGDT connectivity is produced instead;
 - `<output_prefix>info.csv`, containing some summary information like the curve length and the running time;
 - `<output_prefix>cycle.obj`, containing the exact geodesic lines representing the reconstructed curve. If a Hamiltonian cycle is not found, this file is not produced.

The file `visualization/pick_samples.py` contains a Python script for Blender for helping in picking samples.  
After selecting the desired mesh, enable the edit mode, shift+select the samples, and run the script.


### Reproducibility
This repository contains the data for reproducing the experiment depicted in the teaser figure of the paper.  
The experiment can be reproduced by running the executable without arguments (from the `build` directory) or by giving the file `examples/teaser.json` as argument.


## Visualization
All the output files can be imported into Blender "as-is".  
The vertices can be visualized using a `Geometry Node` modifier and plugging the `Mesh to Points` node between the input and the output.  
The curves can be visualized with `right click -> convert to -> curve` and the giving them a depth from `Object data properties -> geometry -> bevel -> depth`.


## License
This repository is distributed under the MIT license and makes use of the following dependencies:
 - [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), distributed under the [MLP-2.0](https://www.mozilla.org/en-US/MPL/2.0/) license;
 - [Geometry Central](https://geometry-central.net/), distributed under the [MIT](https://opensource.org/license/mit) license;
 - [JSON for Modern C++](https://json.nlohmann.me/), distributed under the [MIT](https://opensource.org/license/mit) license.