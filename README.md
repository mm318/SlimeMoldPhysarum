# Slime Mould: 2D Propagation

An openFrameworks animation of slime agents exploring an obstacle-free surface
from `(0.08, 0.5)` toward food at `(0.92, 0.5)`.

## Simulation

Each agent has just two position coordinates, x and y, on the unit square.
Every agent starts at the exact same point, with a seeded random heading.
**Each movement is at most `0.01` in Euclidean distance**, including exploratory
turns and the final movement onto the food. The limit is per simulation step,
not per coordinate or rendered frame. Food is `0.84` units away, so reaching it
requires at least 84 steps. Arrived agents stop at the exact food position.

This is a **local-motion adaptation of SMA**, rather than the original
unconstrained optimizer. Fitness is distance to food. SMA's fitness ranking,
`log10` weights, best-agent/peer influence, and shrinking oscillations steer
persistent headings. A food-distance field supplies the direction toward food,
with stronger attraction as exploration subsides. Random restarts change
headings instead of relocating agents. All movement is bounded before it is
recorded. The simulation ends when all agents arrive or the step limit is met.

Green growth shows the surface the agents actually traversed. Overlapping
tracks build brighter golden veins; light dots mark growing tips. Deposits
are softly blurred for an organic appearance, while exploration history remains
visible without evaporation. The optional closest-trail overlay follows one
agent's real trajectory from the origin. There are no precomputed connections
or trails drawn to food before agents travel there.

The sidebar shows arrivals, closest distance, actual trail length, and progress.
The shortest arrival trail is the shortest among the recorded successful agents;
movement optimizes distance to food, not total trajectory length.
Defaults: 240 agents, 1,200 steps, direction-reset probability `0.03`, and seed `2026`.

## Controls

| Key | Action |
| --- | --- |
| Space | Pause/resume |
| N | Advance one simulation step and pause |
| R | Restart with a new seed |
| Enter | Repeat the current seed |
| S / L | Save/load controls |

The panel toggles exploration trails, growing tips, and the closest agent's
trail. Changing population, step limit, or direction-reset probability restarts
from the origin with the current seed. Steps per frame controls playback speed;
it does not increase the `0.01` movement cap per step. Settings are saved to
`build/bin/data/propagation-settings.xml`.

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

Tests verify identical starting positions, two-dimensional exploration,
Euclidean step limits throughout every recorded trajectory, local resets,
exact food arrivals, no arrival before step 84, deterministic seeds, bounded
positions, consistent trail lengths, and stopping/reset behavior. Both zero
and 100% direction-reset probabilities are exercised.

The simulation uses only the C++ standard library. Its tests can also run
without openFrameworks or a display:

```bash
c++ -std=c++17 -O2 -Isrc tests/sma.cpp src/SmaOptimizer.cpp -o /tmp/slime-tests
/tmp/slime-tests
```

## Sources

These sources describe the same original 2020 SMA optimizer, which supplies
the ranked weights and oscillation model adapted here:

- [MathWorks File Exchange](https://in.mathworks.com/matlabcentral/fileexchange/76619-slime-mould-algorithm-sma-a-method-for-optimization)
- [Authors' page, paper, and MATLAB code](https://aliasgharheidari.com/SMA.html)
- [Baeldung explanation](https://www.baeldung.com/cs/slime-mould-algorithm)

Li, Chen, Wang, Heidari, and Mirjalili, *Slime mould algorithm: A new method for
stochastic optimization*, Future Generation Computer Systems 111 (2020),
300–323. [DOI: 10.1016/j.future.2020.03.055](https://doi.org/10.1016/j.future.2020.03.055).
