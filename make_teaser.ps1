git submodule update --init --recursive --remote

Copy-Item ".\deps\win-fix\geometrycentral-deps\CMakeLists.txt" -Destination ".\deps\geometry-central\deps"

mkdir build
cd build
cmake ..
cmake --build . --config release --parallel

./bin/CurvesSurf.exe ../examples/teaser.json