# Slime Mold Physarum

An openFrameworks slime mold simulation using OpenGL compute shaders.

## Colonies, routes, and energy

There are 25 teams. Each team starts with a randomly chosen 2 or 3 permanent
sites and a fixed population of 1,536 agents (38,400 agents total). Its agents
start uniformly distributed inside small disks around those sites. Resetting
with **R** creates new sites and colonies together.

Environmental sensing uses **local pheromones**. Scouts start with random headings and
make persistent random walks. Each colony has its own scent within its team's
field. Sites emit that scent locally, and departing agents spread it as they
explore. It diffuses and evaporates. Agents prefer fresh space and respond to
locally sampled scent from their team's other colonies. Discovery depends on
exploration and overlapping scent trails; it takes longer than direct homing.

Each agent keeps its originating site as a fixed home and alternates between
outbound exploration and returning home. **Outbound actions** sets the base
outing length (default 96 paid actions). Initial limits vary from 1 to that
value; later outings vary from half to one-and-a-half times it. Agents therefore
return and explore at different times. Waiting for energy does not advance
their outbound clock. Discovering another site can start a return early.

Returners navigate their own recorded breadcrumbs and local home scent. Within
their local sensor radius, they can skip remembered bends when the direct
connection is shorter and cheaper, and can detour around hostile trails. They
do not steer toward unvisited site coordinates. Arriving home starts another
outing with a new heading; agents physically travel back and are never
teleported or respawned. The fixed population keeps traffic flowing back to the
sites while local shortcuts can improve existing paths.

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
enemy trail. Developing trails remain visible below that threshold. Site visits
do not certify or protect a route.

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
| Explore or return through free/friendly space | 1 |
| Enter another team's weak pheromone | 3 |
| Attack another team's established route | 12 |
| Defend one attacked route segment | 8 |

The pool can fund any subset of the team's agents. Agents that cannot afford
their next action wait; they do not die. Allocation order rotates each turn.
Unused energy expires at the end of the turn. All four costs are independently
configurable positive integers; the defaults make hostile actions more expensive.

Each turn funds actual attacks first, then resolves active defense, then funds
ordinary exploration and return movement. This protects defense from having
its energy exhausted by ordinary movement. A team's own attacks can still
consume energy it might otherwise have spent defending. There is one shared
pool, with no separate defense allowance. Only paid attacks trigger a defense
decision.

An established route is a trail above the adjustable traffic-density threshold.
Attackers wait at its border and spend energy to attack the next segment. Its
owner makes one team-level defense decision per threatened segment per turn;
nearby friendly agents are not required. **Defend probability** is configurable
from 0 to 1 (default 0.5):

- If defense is chosen and the team can pay **Defense cost**, it pays once and
  successfully blocks every attack targeting that segment for that turn.
- If defense is declined, it costs nothing and supplies no resistance.
- If defense is chosen but unaffordable, it fails and charges no defense energy.

Without a funded defense, paid attackers erase the segment. Route strength does
not provide free passive resistance. Attack energy is spent even when defense
blocks the attack. Paid defense prevents attack damage; ordinary evaporation
and diffusion still apply. Colony scents and permanent sites remain.

The sidebar controls energy, movement, scent, and combat. **Space** pauses,
**N** advances one turn, and **S / L** saves or loads controls in
`build/bin/data/colony-settings.xml`. The statistics describe the latest sampled
turn, not cumulative totals. Defense successes, failures, and clears count
segments; attack counts count agents. The explore/return counts include agents
waiting for energy. Simulation speed is measured in turns; changing
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
builds synchronize shader edits and deletions even when no C++ files require
recompilation. Saved control settings are preserved.
The openFrameworks installation is left unchanged. A graphical session and an
OpenGL 4.4-capable driver are required by the bundled GLSL 4.40 shaders. This
ofxCMake revision supports desktop Linux. The app uses X11 (XWayland in a
Wayland session), matching openFrameworks 0.12.1's Linux window backend.

## Verification

The default build includes GPU integration checks for colony initialization,
reproducible resets, independent team fields, pooled energy limits, terrain
costs, combat outcomes, route erasure, exploration, independence from distant
site positions, partial networks, intermediate-node routes, ordinary route decay,
voluntary abandonment, staggered return cycles, physical home arrivals, local
shortcuts, paid probabilistic defense, independent action costs, local
scent-guided connections, and separate rendering layers:

```bash
ctest --test-dir build --output-on-failure
```

Tests use a hidden OpenGL window. If `xvfb-run` is available, CMake runs them in
a virtual display with software rendering; otherwise a graphical session is
required. Disable the test executable with `-DBUILD_TESTING=OFF` if desired.
