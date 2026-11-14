# Physarum: Growing Flow Networks

An openFrameworks simulation of a branching transport graph, approximating the
behavior shown in [One Seed Builds a Living Network][growth-video]
by Rule & Flow. The creator describes a local-flow graph simulation; their exact
code and growth rules are not available. This implementation is an inferred
model, not a reproduction of their source code.

## Scenario

Every food starts an independent growing colony, with a permanent root and no
edges. Food A is at `(0.08, 0.5)` and food B is at `(0.92, 0.5)`. The default
six-food scene adds four sources across the unit square. Disable **Video scene:
six foods** to use only A and B. There are no obstacles.

Every branch extension and merging connection is at most **`0.01` units in
Euclidean length**. Opposing fronts can meet between food sources. Colonies
merge only through physical contact; the graph starts without a hidden
skeleton or a precomputed route.

## Graph growth and transport

The default tip budget gives each food three initial tips. Persistent tips
extend short segments, fork, and have a modest bias toward other foods.
Separate colonies take priority when selecting a target; a tip can finish
pursuing that target after the colonies connect. Food sites replenish missing
frontiers in rotation, sharing the global tip cap. With fewer slots than foods,
some foods wait for a slot. Established, food-carrying trunks can sprout lateral
probes as slots become available.

A crossing splits an existing tube into a real hydraulic junction; tips can
also join nearby nodes. Contact merges separate colonies or closes a loop in
an existing colony. When two exploring tips meet, both tips finish and their
connecting paths become eligible for hydraulic adaptation. When a tip meets
the interior of another exploring branch, only the incoming tip finishes.
The other tip keeps growing, with protection limited to the distal portion
beyond the new junction. The proximal paths become a transport route.
Forks retain protection on paths still leading to surviving tips.

Both routes of a loop remain available for hydraulic competition. Food
approaches and nearby-node joins check their actual connecting segment for
intervening tubes, including head-on collinear contacts. Established nodes
stay fixed.

Growth has no step or duration limit. The node budget counts only the current
live network, including every permanent food root. When tubes retire, fragments
with no remaining connection to any food are removed, freeing capacity.
Separate colonies remain alive while supplied by their own foods. Shared
junctions remain while another live tube uses them.

A full budget pauses extensions while existing food-to-food transport and tube
adaptation continue. Protected frontiers wait for room while unused trunks
can weaken and free capacity. A budget occupied entirely by protected growth
can stay full until frontiers are removed or the budget is increased. Finished
tips release slots for new frontiers.

Each tube has a length `L`, conductivity `D`, and signed flow `Q`. At every
step, a sparse, grounded graph-Laplacian solve computes node pressures and
flows:

```text
Q(a,b) = D(a,b) / L(a,b) * (pressure(a) - pressure(b))
sum(outgoing flows at a node) = supplied flow - consumed flow
```

Each disconnected component has its own pressure reference and must balance
its supplies and sinks. There is no hydraulic flow between colonies until
they physically join. Food-to-food flow selects the backbone: conductivity
adapts toward the average `|Q|` over **every food pair within its component**,
using the linear response for both two and six foods. Root-to-food pressure
solutions provide every pair's flow by superposition. Displayed pumping pairs
and directions can change without changing reinforcement.

Exploring branches carry **zero flow** and create no artificial sinks. Their
conductivity stays at `D = 0.003`, protected from hydraulic decay while they
lead to active tips. Exploration continues after the food colonies connect,
including lateral probes from established trunks. It is bounded by the tip and
live-node caps, with no age, distance, or step-count limit for a growing branch.

After contact, the newly established route loses exploration protection and
its flow follows its resistance and the solved pressures. Food transport alone
determines its thickening or thinning. Weak tubes must remain weak for 24
consecutive adaptation steps before removal. Thin routes gaining useful
transport survive that pruning check. Abandoned branches also lose protection.
Protected pendant trees are omitted from the hydraulic solve and inherit their
anchor's pressure, preserving the zero-flow relation along every segment.

Route shortening comes from **mesh formation and hydraulic selection**. When
growth reconnects with an existing trunk, flow divides between the old route
and the new connection according to their resistances. Useful connections
thicken; inefficient portions progressively lose conductivity and retire. Their
unused nodes release capacity for further growth. Established nodes stay fixed:
there is no geometric relaxation or string-tightening step. A shorter route can
only emerge if local exploration actually grows the necessary connections.
The result depends on the mesh explored; a sparse graph need not contain the
straight-line optimum.

The hydraulic solve uses Jacobi-preconditioned conjugate gradients, with a
larger convergence budget for the conductance contrasts produced by thinning
tubes. Flow reinforcement changes actual transport, not just the appearance.

Gold strokes highlight food transport, with width mapped from conductivity;
thin green strokes show active scouts.
Animated bright points follow the signed flow direction. Gray lines retain
explored and retired branches; small points mark live graph nodes. Retired
geometry is a separate, bounded drawing history (20,000 segments), so it does
not consume the live-node budget or hydraulic solver capacity. The progress
chart retains the last 4,096 samples without limiting simulation duration.
Every food ring marks a permanent colony root. The sidebar reports the number
of separate food groups, graph size, live tube length, food-carrying backbone
length, and its shortest A-B route. One food group means all colonies are
joined.
Weak scout connections do not count as an established backbone route.
Dijkstra's algorithm is used only to report that route length, not to guide
growth or reinforcement.

The chart records the closest historical gap between the A and B colonies,
reaching zero when they join. Growth and exploration protection are custom
choices for this visualization.
This is neither the 2020 numerical SMA optimizer nor a biological validation
or proof of fractal scaling. Multi-food adaptation can favor shared routes
over the shortest route between one particular food pair.

## Controls

| Key | Action |
| --- | --- |
| Space | Pause/resume |
| N | Advance one step and pause |
| R | Restart with a new seed |
| Enter | Repeat the current seed |
| S / L | Save/load controls |

The panel selects two or six foods, toggles graph/flow layers, and controls
branching, nutrient bias, live-node budget, tip limit, and
hydraulic adaptation.
**Max growing tips** is adjustable live from **1 to 128** without resetting the
network. Lowering it immediately removes excess frontiers; raising it permits
additional forks, lateral probes, and food-site restarts. Other model changes
restart growth; visibility and playback speed preserve state. Settings are
saved to `build/bin/data/graph-settings.xml`.

Defaults: seed `2026`, continuous simulation, 5,000 live nodes, and 18 growing
tips. Pause manually with Space. The `0.01` extension limit applies to each
simulation step, independently of the steps-per-frame playback setting.

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

Tests check analytic parallel-tube flows, supply reversal, independent pressure
references, and rejection of flow between disconnected colonies. Controlled
two- and six-food meshes verify that a thin shorter route can replace a thick
detour, while an inefficient newcomer cannot displace the shorter route.
Independent pairwise solves check the production food-flow average.

Controlled contacts verify that tip-to-tip fusion consumes both tips and an
interior branch contact preserves only the other tip's distal exploring subset.
They check zero flow in protected segments and real flow in the new trunk.
End-to-end checks cover growth from every food, fixed node positions, physical
step lengths, mass conservation, continued exploration, lateral probes, meshes,
pruning, and deterministic resets. A 5,000-step regression explicitly retires
frontiers to repeatedly reuse a 25-node capacity. Live tip-limit tests verify
that changing the cap preserves the graph. A single-tip case checks that
flow-free branches outlive 31 steps, connect, and rotate food origins.

The graph simulation uses only the C++ standard library and can be tested
without openFrameworks or a display:

```bash
c++ -std=c++17 -O2 -Isrc tests/physarum.cpp src/PhysarumSimulation.cpp \
    -o /tmp/physarum-tests
/tmp/physarum-tests
```

## References

- [Railway-network reference and model explanation][railway-video].
- [Reference video and creator description][growth-video].
- [Physarum Can Compute Shortest Paths](https://arxiv.org/abs/1106.0423),
  Bonifaci, Mehlhorn, and Varma: hydraulic flow and adaptive conductance in the
  Tero model.

Classical adaptive-conductance models operate on an existing graph. The local
growth, bifurcation, joining, exploration protection, rotating food supply, and
visual presentation here are additions inferred for this project.

[growth-video]: https://www.youtube.com/shorts/PGH4C2rPiaM
[railway-video]: https://www.youtube.com/watch?v=Jt8qA-9z7XI
