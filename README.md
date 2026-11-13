# Slime Mould Algorithm: 2D Path Search

An openFrameworks visualization of the Slime Mould Algorithm (SMA), optimizing
a route between two fixed points on a 2D surface.

## Scenario

The unit-square surface has endpoints A `(0.08, 0.5)` and B `(0.92, 0.5)`.
A disk at `(0.5, 0.5)` with radius `0.17` blocks the direct connection; routes
must maintain `0.012` clearance. Disable the obstacle to test the straight-line
optimum of length `0.84`.

Each search agent represents a complete route with eight interior waypoints at
fixed x stations. SMA optimizes their y coordinates. The score is route length
plus `100 + 1000 * penetration depth` per intersecting segment/obstacle pair.
Collision checks cover entire segments, including the required clearance.

Faint lines show candidate routes. The bright line shows the best route found:
yellow-green when feasible, orange when colliding. The sidebar reports length,
score, iteration, seed, and convergence. Defaults: 80 agents, 600 iterations,
restart probability `0.03`, and seed `2026`.

SMA uses fitness-ranked weights, best-so-far attraction, shrinking oscillations,
and random restarts. This implementation follows equations 2.1–2.7 of the
original paper, using the MATLAB reference's `log10` weights and in-place
updates. Iterations start at one to avoid infinite `atanh`; restarts sample
coordinates independently, and the final generated population is evaluated.

The result is the best path found, with no guarantee of global optimality.
Fixed x stations limit routes to paths that advance monotonically in x.

## Controls

| Key | Action |
| --- | --- |
| Space | Pause/resume |
| N | Advance one iteration and pause |
| R | Restart with a new seed |
| Enter | Repeat the current seed |
| O | Toggle the obstacle |
| S / L | Save/load controls |

Changing population, iteration limit, restart probability, or the obstacle
restarts the search with the current seed. Iterations per frame controls
playback speed. Settings are saved to `build/bin/data/sma-settings.xml`.

## Build and run

Requires openFrameworks 0.12.1 with its Linux development dependencies, CMake
3.16+, Git, pkg-config, a C++17-capable compiler, and an OpenGL 3.2 display.
The build uses [ofxCMake](https://github.com/mm318/ofxCMake), pinned to revision
`7a92d86054eb41a75584b2ddedcc0c72a3c2f325`, and the bundled ofxGui addon.

The default openFrameworks location is `../of_v0.12.1_linux64_gcc6_release`.
From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 4
./build/bin/Slime_Mold
```

For another installation, add `-DOF_DIRECTORY_BY_USER=/path/to/openFrameworks`
to the configure command. For an offline build, also set
`-DFETCHCONTENT_SOURCE_DIR_OFXCMAKE=/path/to/ofxCMake` to a local checkout.
Build outputs stay in `build/`; the app requires no bundled assets.

## Tests

```bash
ctest --test-dir build --output-on-failure
```

The tests cover SMA convergence, deterministic seeds, bounds, stopping,
restarts, collision geometry, clearance, and the empty-surface optimum.
Three fixed seeds must produce obstacle detours within 1% of the continuous
shortest path (two tangents joined by an arc around the clearance-inflated disk).

The optimizer and geometry use only the C++ standard library. Their tests can
also run without openFrameworks or a display:

```bash
c++ -std=c++17 -O2 -Isrc tests/sma.cpp src/SmaOptimizer.cpp src/PathProblem.cpp -o /tmp/sma-tests
/tmp/sma-tests
```

## Sources

These sources describe the same 2020 SMA optimizer:

- [MathWorks File Exchange](https://in.mathworks.com/matlabcentral/fileexchange/76619-slime-mould-algorithm-sma-a-method-for-optimization)
- [Authors' page, paper, and MATLAB code](https://aliasgharheidari.com/SMA.html)
- [Baeldung explanation](https://www.baeldung.com/cs/slime-mould-algorithm)

Li, Chen, Wang, Heidari, and Mirjalili, *Slime mould algorithm: A new method for
stochastic optimization*, Future Generation Computer Systems 111 (2020),
300–323. [DOI: 10.1016/j.future.2020.03.055](https://doi.org/10.1016/j.future.2020.03.055).
