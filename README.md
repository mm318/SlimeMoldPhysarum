# Physarum: Growing Flow Networks

An openFrameworks simulation of a branching transport graph, approximating the
behavior shown in [One Seed Builds a Living Network](https://www.youtube.com/shorts/PGH4C2rPiaM)
by Rule & Flow. The creator describes a local-flow graph simulation; their exact
code and growth rules are not available. This implementation is an inferred
model, not a reproduction of their source code.

## Scenario

The graph begins with a single seed at food A `(0.08, 0.5)` and no edges. Food B
is at `(0.92, 0.5)`. Both are permanent nutrient sites. The default six-food scene
adds four sources across the unit square, as a comparison with the video.
Disable **Video scene: six foods** to use only A and B. There are no obstacles.

Every branch extension and merging connection is at most **`0.01` units in
Euclidean length**. New tips branch from existing nodes. Food becomes part of
the hydraulic network only when physical growth reaches its exact position.
The graph never begins with a hidden skeleton or a precomputed route.

## Graph growth and transport

Persistent tips extend short segments, occasionally sprout side branches, and
have a modest directional bias toward unreached food. A crossing splits an
existing tube into a real junction; nearby tips can also join existing nodes.
Growth stops at the growth-step or node budget, after which transport continues
to adapt the network.

Each tube has a length `L`, conductivity `D`, and signed flow `Q`. At every step,
a sparse, grounded graph-Laplacian solve computes node pressures and flows:

```text
Q(a,b) = D(a,b) / L(a,b) * (pressure(a) - pressure(b))
sum(outgoing flows at a node) = supplied flow - consumed flow
```

Growing tips draw a small exploratory flow. Discovered foods exchange transport;
each takes a turn supplying the other foods, so neither A nor B is a permanently
absorbing endpoint. A tube's conductivity adapts toward `reinforcement * |Q|^1.1`.
Low-flow tubes narrow and eventually retire; useful routes retain larger
conductivity. This feedback changes actual network transport, not just colors.
The pressure solve uses Jacobi-preconditioned conjugate gradients.

Gold strokes are conducting edges, with width mapped from conductivity.
Animated bright points follow the signed flow direction. Gray lines retain
explored and retired branches; small points mark the graph nodes. Food rings
brighten when reached. The sidebar reports graph size, connected tube length,
and the shortest A-B route remaining in the current graph. Dijkstra's algorithm
is used only to report that route length, not to guide growth or reinforcement.

Growth rules and the `1.1` reinforcement exponent are custom choices for this
visualization. This is neither the 2020 numerical SMA optimizer nor a biological
validation or proof of fractal scaling. Multi-food adaptation can favor shared
routes over the shortest route between one particular food pair.

## Controls

| Key | Action |
| --- | --- |
| Space | Pause/resume |
| N | Advance one step and pause |
| R | Restart with a new seed |
| Enter | Repeat the current seed |
| S / L | Save/load controls |

The panel selects two or six foods, toggles graph/flow layers, and controls
branching, nutrient bias, growing-tip and node budgets, growth duration, and tube
adaptation. Model changes restart the simulation; visibility and playback speed
preserve state. Settings are saved to `build/bin/data/graph-settings.xml`.

Defaults: seed `2026`, 1,200 steps, up to 400 growth steps, 1,800 nodes, and 18
growing tips. The `0.01` extension limit applies to each simulation step,
independently of the steps-per-frame playback setting.

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

Tests check a controlled parallel-tube network against analytic flows, reverse
food supply roles, and verify that reinforcement moves transport to the shorter
competing route. End-to-end checks cover node-level mass conservation, physical
flow/conductivity consistency, local growth and junction lengths, connected
exploration history, exact food discovery, retained food connections, branching,
reconnection, pruning, deterministic resets, and growth/iteration budgets.

The graph simulation uses only the C++ standard library and can be tested
without openFrameworks or a display:

```bash
c++ -std=c++17 -O2 -Isrc tests/physarum.cpp src/PhysarumSimulation.cpp -o /tmp/physarum-tests
/tmp/physarum-tests
```

## References

- [Reference video and creator description](https://www.youtube.com/shorts/PGH4C2rPiaM).
- [Physarum Can Compute Shortest Paths](https://arxiv.org/abs/1106.0423), Bonifaci,
  Mehlhorn, and Varma: hydraulic flow and adaptive conductance in the Tero model.

Classical adaptive-conductance models operate on an existing graph. The local
growth, bifurcation, joining, exploratory demand, rotating food supply, and
visual presentation here are additions inferred for this project.
