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
Growth has no step or duration limit. The node budget counts only the current
live network. When a tube retires, nodes with no remaining live connection are
removed from the working graph, freeing capacity. Shared junctions remain while
another live tube still uses them. The seed always occupies one slot.

A full budget pauses extensions while existing food-to-food transport and tube
adaptation continue. Blocked frontiers stop drawing exploratory growth flow, so
unused branches can weaken. Growth resumes when pruning frees space. Surviving
tips wait for room; if all tips die, reached food sites can launch new ones.

Each tube has a length `L`, conductivity `D`, and signed flow `Q`. At every step,
a sparse, grounded graph-Laplacian solve computes node pressures and flows:

```text
Q(a,b) = D(a,b) / L(a,b) * (pressure(a) - pressure(b))
sum(outgoing flows at a node) = supplied flow - consumed flow
```

Food-to-food flow selects the backbone. Conductivity adapts toward the average
response over **every connected food pair**, calculated by superposing root-to-food
pressure solutions. The displayed pumping pair and flow direction can change
without changing which routes receive reinforcement. Two foods use the linear
Tero response `f(Q) = |Q|`; multiple foods use a saturating response
`f(Q) = 2 * |Q|^1.8 / (1 + |Q|^1.8)`, normalized so `f(1) = 1`.

Exploration receives a separate, limited support flow only while food remains
unconnected. Its conductivity contribution is capped at `0.01` and disappears
once all foods are joined. New segments begin as thin trials (`D = 0.003`), and
unsupported ones retire. A candidate with increasing useful transport is not
removed merely for starting thin. Younger routes receive no extra food-flow
reward. This lets a shorter new route compete while an inefficient new route
cannot replace a better established trunk just by attracting an exploratory tip.

Mature food-carrying tubes can also physically shorten. A local mechanical
relaxation reduces flow-weighted length, with food locations and growing tips
fixed. Moves are gradual, stay within the surface, preserve the `0.01` edge
limit, and reject new crossings. **Tube shortening** can be set to zero to
compare against the original fixed skeleton. This is an additional geometric
model; the reference film's fixed fine mesh selects routes without moving nodes.

The hydraulic solve uses Jacobi-preconditioned conjugate gradients, with a
larger convergence budget for the conductance contrasts produced by contracting
tubes. Flow reinforcement changes actual transport, not just the appearance.

Gold strokes highlight food transport, with width mapped from conductivity;
thin green strokes show active scouts.
Animated bright points follow the signed flow direction. Gray lines retain
explored and retired branches; small points mark live graph nodes. Retired
geometry is a separate, bounded drawing history (20,000 segments), so it does
not consume the live-node budget or hydraulic solver capacity. The progress
chart retains the last 4,096 samples without limiting simulation duration. Food rings
brighten when reached. The sidebar reports graph size, connected tube length,
the food-carrying backbone length, and its shortest A-B route. Weak scout
connections do not count as an established backbone route. Dijkstra's algorithm
is used only to report that route length, not to guide growth or reinforcement.

Growth, exploration support, and geometric shortening are custom choices for
this visualization. This is neither the 2020 numerical SMA optimizer nor a biological
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
branching, nutrient bias, live-node budget, tip limit, tube shortening, and
hydraulic adaptation.
**Max growing tips** is adjustable live from **1 to 128** without resetting the
network. Lowering it immediately removes excess frontiers; raising it permits
additional forks and food-site restarts. Other model changes restart growth;
visibility and playback speed preserve state. Settings are saved to `build/bin/data/graph-settings.xml`.

Defaults: seed `2026`, continuous simulation, 1,800 live nodes, and 18 growing
tips. Pause manually with Space. The `0.01` extension limit applies to each simulation step,
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
competing route regardless of which started thicker. They compare mature
trunk shortening against a fixed skeleton and independently check food-pair
averaging. An aggressive continuous-exploration fixture also verifies that
the two-food backbone stays near the straight-line optimum and settles instead
of cycling through inefficient new branches. End-to-end checks cover node-level mass conservation, physical
flow/conductivity consistency, local growth and junction lengths, connected
working topology, exact food discovery, retained food connections, branching,
reconnection, pruning, deterministic resets, and live-node budgets. A 5,000-step
regression repeatedly reuses a 25-node capacity, and live tip-limit tests verify
that changing the cap preserves the existing graph.

The graph simulation uses only the C++ standard library and can be tested
without openFrameworks or a display:

```bash
c++ -std=c++17 -O2 -Isrc tests/physarum.cpp src/PhysarumSimulation.cpp -o /tmp/physarum-tests
/tmp/physarum-tests
```

## References

- [Railway-network reference and model explanation](https://www.youtube.com/watch?v=Jt8qA-9z7XI).
- [Reference video and creator description](https://www.youtube.com/shorts/PGH4C2rPiaM).
- [Physarum Can Compute Shortest Paths](https://arxiv.org/abs/1106.0423), Bonifaci,
  Mehlhorn, and Varma: hydraulic flow and adaptive conductance in the Tero model.

Classical adaptive-conductance models operate on an existing graph. The local
growth, bifurcation, joining, exploratory demand, rotating food supply, and
visual presentation here are additions inferred for this project.
