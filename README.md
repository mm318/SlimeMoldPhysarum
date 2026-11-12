# Slime Mold Physarum

An openFrameworks slime mold simulation using OpenGL compute shaders.

## Colonies, routes, and energy

There are 25 teams. Each team starts with a randomly chosen 2 or 3 permanent
sites and a fixed population of 1,536 agents (38,400 agents total). Its agents
start uniformly distributed inside small disks around those sites. Resetting
with **R** creates new sites and colonies together.

Movement uses **local pheromones only**. Scouts start with random headings and
make persistent random walks. Each colony has its own scent within its team's
field. Sites emit that scent locally, and departing agents spread it as they
explore. It diffuses and evaporates. Agents prefer fresh space and respond to
locally sampled scent from their team's other colonies. Discovery depends on
exploration and overlapping scent trails; it takes longer than direct homing.

Routes emerge from continuous traffic reinforcement. Every successful movement
deposits trail pheromone, and every agent can follow those trails, regardless of
whether it has visited another site. Repeated traffic strengthens a path;
diffusion and evaporation weaken it when agents move elsewhere. Partial
connections, temporary branches, and junctions between intermediate locations
can all form routes. Connecting two of three sites is sufficient to maintain
that connection while the remaining site is still undiscovered.

Trail reinforcement has the same evaporation rate as colony scent. Routes can
be abandoned and fade without enemy attacks. **Strong trail density** (default
3) controls the bright route rendering and the higher cost of attacking a busy
enemy trail. Developing trails remain visible below that threshold. At contact
with another site, agents reverse their travel direction and respond to the
previous colony's local scent; site visits do not certify or protect a route.

| Appearance | Meaning |
| --- | --- |
| Faint colored mist | Locally diffusing colony pheromone |
| Colored paths that brighten with traffic | Developing trails |
| Solid team color with a dark edge | Strongly reinforced route |
| Team-colored dots with dark outlines | All active agents, including attackers |
| Dim team-colored dots | Agents waiting for energy |
| Labeled colored rings | Permanent sites, e.g. `12A`, `12B` |

The sidebar has independent visibility switches for pheromones, routes, and
agents. **Other colony scent** controls the response to nearby scent gradients;
**Explore fresh space** encourages leaving already explored areas, and
**Exploration turning** controls random wandering. **Local sensor distance**
sets how far ahead agents sample. These controls replace direct site attraction.
**Follow traffic trails** sets the local attraction to reinforced paths.

Every simulation turn refreshes a shared energy pool for each team. The default
budget is 768, and action costs are:

| Action | Energy |
| --- | ---: |
| Explore free space or follow your own scent | 1 |
| Enter another team's weak pheromone | 3 |
| Attack another team's established route | 12 |

The pool can fund any subset of the team's agents. Agents that cannot afford
their next action wait; they do not die. Allocation order rotates each turn.
Unused energy expires at the end of the turn. Controls preserve the ordering
`exploration < foreign pheromone < attack`.

An established route is a trail above the adjustable traffic-density threshold.
Attackers wait at its border and spend energy to attack the next segment. A
contest resolves simultaneous attacks on each targeted route segment: attacking
effort competes against route strength and current reinforcement. Either side
can win. A successful attack erases the defending route segment; a failed attack
spends energy without erasing it. Colony scents and permanent sites remain.

The sidebar controls energy, movement, scent, and combat. **Space** pauses,
**N** advances one turn, and **S / L** saves or loads controls in
`build/bin/data/colony-settings.xml`. The statistics describe the latest sampled
turn, not cumulative totals. Simulation speed is measured in turns; changing
the turns-per-frame control accelerates it.

## Build on Linux

The CMake build uses [mm318/ofxCMake](https://github.com/mm318/ofxCMake), pinned
to revision `7a92d86054eb41a75584b2ddedcc0c72a3c2f325`, and openFrameworks
0.12.1. It requires CMake 3.16+, Git, pkg-config, and a C++17-capable compiler.
ofxCMake selects C++20 by default with GCC 10 or newer.

Use the existing extracted release at
`/home/user/of_v0.12.1_linux64_gcc6_release`; no new openFrameworks download
or separate core build is needed. The default path is that release folder next
to this repository. Configure and build from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DOF_DIRECTORY_BY_USER=/home/user/of_v0.12.1_linux64_gcc6_release
cmake --build build --parallel 4
./build/bin/Slime_Mold
```

The first configure fetches the pinned ofxCMake revision. To use the existing
local checkout instead (including when offline), add the following to the
configure command:

```bash
-DFETCHCONTENT_SOURCE_DIR_OFXCMAKE=/home/user/ofxCMake
```

For another installation, set `OF_DIRECTORY_BY_USER` to its extracted official
0.12.1 Linux release directory. If system development libraries are missing,
install the openFrameworks dependencies using the distribution's script, for
example on Ubuntu:

```bash
sudo /home/user/of_v0.12.1_linux64_gcc6_release/scripts/linux/ubuntu/install_dependencies.sh
```

The build compiles the openFrameworks core, the release's `ofxGui` addon, and
the application together. Generated libraries stay in `build/lib`; the
executable and a copy of `bin/data` (shaders) go in `build/bin`. Subsequent
builds also copy shader edits when no C++ files require recompilation.
The openFrameworks installation is left unchanged. A graphical session and an
OpenGL 4.4-capable driver are required by the bundled GLSL 4.40 shaders. This
ofxCMake revision supports desktop Linux. The app uses X11 (XWayland in a
Wayland session), matching openFrameworks 0.12.1's Linux window backend.

## Verification

The default build includes GPU integration checks for colony initialization,
reproducible resets, independent team fields, pooled energy limits, terrain
costs, combat outcomes, route erasure, exploration, independence from distant
site positions, partial networks, intermediate-node routes, ordinary route decay,
voluntary abandonment, local scent-guided connections, and separate rendering layers:

```bash
ctest --test-dir build --output-on-failure
```

Tests use a hidden OpenGL window. If `xvfb-run` is available, CMake runs them in
a virtual display with software rendering; otherwise a graphical session is
required. Disable the test executable with `-DBUILD_TESTING=OFF` if desired.
