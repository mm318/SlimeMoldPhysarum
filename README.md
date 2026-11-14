# Physarum: Growing Flow Networks

An openFrameworks simulation of ten competing branching transport networks.
Growth and hydraulic adaptation draw inspiration from
[One Seed Builds a Living Network][growth-video] by Rule & Flow. The creator's
exact code and growth rules are unavailable; competition and ownership here
are custom additions to an inferred model.

## Scenario

There are exactly **ten slimes**. Each independently receives **two or three
food sources**, chosen randomly when the scene resets. Food positions are also
random, with spacing to keep initial roots apart. Repeating a seed reproduces
the same food counts, positions, and growth.

Every food is a permanent root belonging to one slime. Each slime has a distinct
color shared by its foods, branches, tips, and retired trails. Labels such as
`3B` identify food B of slime 3. Each food starts with no edges. There are no
fixed obstacles. Explorers can cross foreign explorers, but established
foreign trunks block both new growth and later trunk promotions.

Every branch extension and merging connection is at most **`0.01` units in
Euclidean length**. Fronts belonging to the same slime can meet between its
foods. Different slimes never share nodes, tubes, foods, or hydraulic flow.
The graph starts without a hidden skeleton or precomputed routes.

## Graph growth and transport

The bootstrap gives each food three initial tips, subject to its slime's cap.
After that, foods, trunk sprouts, and frontier forks share one birth budget per
slime. There are no automatic per-food refills or independent per-tip branching
chances. **New tips / step / slime** controls the rate, defaulting to one birth
per step when space and eligible sites are available. Fractional rates accrue
credit; blocked growth cannot accumulate a large future burst.

Birth sites are sampled by available physical length. Each eligible food or
frontier site contributes `0.01` units of weight; carrying trunks contribute
their unblocked length. Subdividing a tube into more graph nodes does not
increase its chance of spawning. All sites use a `0.03` spacing from other
active tips of the same slime; a fork excludes its own parent from that check.
Frontiers need six growth steps between forks. Trunks need useful food flow,
with no additional age gate. Interior births split a tube while preserving its
geometry, conductivity, and hydraulic resistance.

A slime that loses every tip restarts one probe through the same weighted pool,
including when the rate is zero. Zero rate otherwise disables additional
births. An unused straight sprout junction is reclaimed after its probe dies,
preserving the trunk and freeing node capacity. Persistent tips extend short
segments and have a modest bias toward their own slime's foods. Disconnected
foods take priority when selecting a target; a tip can finish pursuing its
target after its colonies connect. Growth order rotates to share the node cap.

Contacts obey ownership:

- Same-slime tips join and both finish, proposing a new transport route.
- An accepted own-branch join finishes the incoming tip; the other tip keeps
  growing on its protected distal portion beyond the junction.
- Foreign explorers pass through each other, including tip-to-tip encounters.
  They can pass isolated foreign food roots without claiming them. Crossings
  never create shared nodes or hydraulic links.
- An extension that would touch a foreign established trunk ends its incoming
  tip immediately. No crossing segment is added, and the foreign trunk and its
  frontiers stay unchanged. Checks use the actual segment up to any earlier
  own-slime junction, not the unused remainder of the proposed heading.

Promotion also checks the entire proposed route, including earlier segments.
This catches crossings made while the foreign route was still exploring,
before it gained trunk priority. The existing trunk keeps its claim.
Only active established foreign trunks block promotion; foreign explorers,
isolated foods, abandoned branches, and retired trails do not reserve space.
Crossing, overlapping, or approaching a foreign trunk within `0.0035` units
blocks new extensions and rejects conflicting promotion proposals.

The first established route keeps priority. A rejected route is annihilated
without entering the hydraulic solve, and its nodes release live capacity.
Explorers that become disconnected from every own food also disappear;
unrelated rooted frontiers survive. Successful promotion does not immediately
kill crossing foreign explorers. They remain alive until their own promotion
attempt, which then encounters the earlier claim. Simultaneous proposals use
a rotating slime order to select a reproducible winner.

A spatial index contains only claimed trunks for promotion checks. Extension
checks skip foreign explorers and discard distant trunk bounding boxes before
testing the remaining geometry.
Forks retain protection on paths still leading to surviving tips. Losing a tip
releases its unshared exploring path for abandonment and eventual retirement.

Within one slime, both routes of a loop remain available for competition. Food
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

Each disconnected component has its own pressure reference and balances its
supplies and sinks. Food-to-food flow selects the backbone: conductivity adapts
toward the average `|Q|` over **every own-food pair within its component**.
Root-to-food pressure solutions provide every pair's flow by superposition.
Displayed pumping pairs and directions can change without changing
reinforcement. Cross-slime edges are rejected by the graph and flow solver.

Exploring branches carry **zero flow** and create no artificial sinks. Their
conductivity stays at `D = 0.003`, protected from hydraulic decay while they
lead to active tips. Exploration continues after a slime's foods connect,
including lateral probes from established trunks. It is bounded by the tip and
live-node caps, with no age, distance, or step-count limit for a growing branch.

After accepted promotion, the new route loses exploration protection and
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

Color identifies the slime. Thick strokes show food transport, thin strokes
show scouts, and faint trails retain exploration history. Animated bright
points follow signed flow. Retired trails are drawing history only and do not
block tips, consume live-node capacity, or enter hydraulic solves. History is
bounded to 20,000 retired segments and 4,096 progress samples per slime.

The sidebar lists all ten slime colors and food counts. **Inspect slime**
selects a slime's connected-food groups, food A-B backbone length, and approach
chart. One food group means that slime's foods are joined. Weak scout links
do not count as established backbone routes. Dijkstra's algorithm reports
route length without guiding growth or reinforcement. The chart records the
closest historical gap between that slime's first two food colonies.

Growth, protection, and competition are custom visualization choices. This is
neither the 2020 numerical SMA optimizer nor biological validation or proof of
fractal scaling. Results depend on explored connections and trunk claims;
three-food adaptation can favor shared routes over a particular pair's shortest
route.

## Controls

| Key | Action |
| --- | --- |
| Space | Pause/resume |
| N | Advance one step and pause |
| R | Restart with a new seed |
| Enter | Repeat the current seed |
| S / L | Save/load controls |

The panel toggles drawing layers and controls the shared birth rate, nutrient
bias, live-node budget, per-slime tip limit, and hydraulic adaptation.
**Max tips per slime** is adjustable live from **1 to 128** without resetting
the scene. Lowering it trims each slime independently; raising it permits forks,
lateral probes, and food-site restarts. Changing the inspected slime,
visibility, or playback speed preserves state. Other model changes restart
growth. Settings are saved to `build/bin/data/graph-settings.xml`.

Defaults: seed `2026`, ten slimes, continuous simulation, **5,000 total live
nodes**, **18 growing tips per slime**, and one new tip per slime per step
when eligible sites and capacity are available. Pause with Space. The `0.01`
extension limit applies to each simulation step, independently of playback
speed.

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
Foreign-contact cases verify passage through explorers and isolated foods,
immediate blocking at foreign trunks, and valid own-slime joins before a
blocker. Promotion cases cover earlier crossings,
rollback and node reclamation, survival of unrelated probes, simultaneous
claims, overlap, endpoint contact, and nearby non-conflicting routes. A
ten-slime run checks random food ownership, own-food targeting,
independent tip caps, permanent sources, mass conservation, and the absence of
cross-slime edges. Isolated fixtures check zero-flow exploration, shared forks,
short-route competition, and junction geometry. A 5,000-step regression
explicitly retires frontiers to repeatedly reuse a 25-node capacity. Seeded
resets reproduce food placement, growth, and flow.

Spawn tests compare identical trunks with different subdivisions and verify
the same physical sampling positions, site proportions, and spacing masks.
They check that food births, forks, and trunk sprouts spend one shared budget,
including fractional rates and restarts without food preference. Trunk-birth
and retirement tests verify preserved flow and reclaimed junction capacity.

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
growth, ownership, contact rules, protection, rotating food supply, and
visual presentation here are additions inferred for this project.

[growth-video]: https://www.youtube.com/shorts/PGH4C2rPiaM
[railway-video]: https://www.youtube.com/watch?v=Jt8qA-9z7XI
