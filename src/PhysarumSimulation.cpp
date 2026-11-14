#include "PhysarumSimulation.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <stdexcept>
#include <utility>

namespace {
using Position = PhysarumSimulation::Position;
Position add(Position a, Position b) { return {a.x + b.x, a.y + b.y}; }
Position subtract(Position a, Position b) { return {a.x - b.x, a.y - b.y}; }
Position scale(Position p, double s) { return {p.x * s, p.y * s}; }
double dot(Position a, Position b) { return a.x * b.x + a.y * b.y; }
double cross(Position a, Position b) { return a.x * b.y - a.y * b.x; }
Position unit(Position p) {
    const double length = std::hypot(p.x, p.y);
    return length > 1e-12 ? scale(p, 1 / length) : Position{1, 0};
}
Position rotate(Position p, double angle) {
    return {p.x * std::cos(angle) - p.y * std::sin(angle), p.x * std::sin(angle) + p.y * std::cos(angle)};
}
bool inside(Position p) { return p.x >= 0 && p.x <= 1 && p.y >= 0 && p.y <= 1; }
using Adjacency = std::vector<std::vector<std::pair<std::size_t, double>>>;
Adjacency adjacency(const std::vector<PhysarumSimulation::Node>& nodes,
                    const std::vector<PhysarumSimulation::Edge>& edges) {
    Adjacency result(nodes.size());
    for (const auto& edge : edges) if (edge.active) {
        const double conductance = edge.conductivity / edge.length;
        result[edge.a].push_back({edge.b, conductance});
        result[edge.b].push_back({edge.a, conductance});
    }
    return result;
}
std::vector<std::size_t> components(const Adjacency& neighbors) {
    std::vector<std::size_t> roots(neighbors.size(), PhysarumSimulation::noNode);
    for (std::size_t root = 0; root < neighbors.size(); ++root) if (roots[root] == PhysarumSimulation::noNode) {
        std::queue<std::size_t> queue;
        queue.push(root); roots[root] = root;
        while (!queue.empty()) {
            const auto node = queue.front(); queue.pop();
            for (const auto& neighbor : neighbors[node]) if (roots[neighbor.first] == PhysarumSimulation::noNode) {
                roots[neighbor.first] = root;
                queue.push(neighbor.first);
            }
        }
    }
    return roots;
}
}

double PhysarumSimulation::distance(Position a, Position b) { return std::hypot(a.x - b.x, a.y - b.y); }
double PhysarumSimulation::uniform() { return std::generate_canonical<double, 53>(random); }
PhysarumSimulation::Position PhysarumSimulation::randomDirection() {
    const double angle = uniform() * 2 * std::acos(-1.0);
    return {std::cos(angle), std::sin(angle)};
}

void PhysarumSimulation::reset(const Settings& s, std::uint32_t seed) {
    if (s.maxNodes < (s.sixFoods ? 6u : 2u) || s.maxNodes > 10000 ||
        s.maxTips == 0 || s.maxTips > 128 || !std::isfinite(s.branching) || s.branching < 0 || s.branching > 1 ||
        !std::isfinite(s.foodBias) || s.foodBias < 0 || s.foodBias > 1 ||
        !std::isfinite(s.adaptation) || s.adaptation <= 0 || s.adaptation > 1 ||
        !std::isfinite(s.reinforcement) || s.reinforcement < 0 || s.reinforcement > 3)
        throw std::invalid_argument("Invalid growing-graph settings");
    settings = s;
    random.seed(seed);
    iteration = 0;
    balanceError = 0;
    nodes.clear(); parents.clear(); edges.clear(); tips.clear(); retired.clear();
    nextFood = 0;
    tips.reserve(settings.maxTips);
    foods = {{start, noNode}, {goal, noNode}};
    if (settings.sixFoods) {
        for (Position p : {Position{0.25, 0.16}, Position{0.72, 0.18},
                           Position{0.3, 0.82}, Position{0.78, 0.8}}) foods.push_back({p, noNode});
    }
    for (auto& food : foods) food.node = addNode(food.position);
    for (unsigned fan = 0; fan < 3; ++fan)
        for (std::size_t f = 0; f < foods.size() && tips.size() < settings.maxTips; ++f)
            launchFoodTip(f, fan);
    nextFood = tips.size() % foods.size();
    injections.assign(nodes.size(), 0);
    history.assign(1, distance(start, goal));
}

void PhysarumSimulation::setTipLimit(unsigned limit) {
    if (limit == 0 || limit > 128) throw std::invalid_argument("Tip limit must be between 1 and 128");
    settings.maxTips = limit;
    tips.reserve(limit);
    if (tips.size() > limit) tips.resize(limit);
    if (!nodes.empty()) { updateExploration(); transport(); }
}

std::size_t PhysarumSimulation::component(std::size_t node) const {
    while (parents[node] != node) node = parents[node];
    return node;
}

void PhysarumSimulation::rebuildComponents() { parents = components(adjacency(nodes, edges)); }

std::size_t PhysarumSimulation::addNode(Position position, double pressure) {
    const auto index = nodes.size();
    nodes.push_back({position, pressure, iteration});
    parents.push_back(index);
    return index;
}

void PhysarumSimulation::launchFoodTip(std::size_t food, unsigned fan) {
    Position center{};
    for (const auto& site : foods) center = add(center, scale(site.position, 1.0 / foods.size()));
    auto direction = iteration == 0
        ? rotate(unit(subtract(center, foods[food].position)), (int(fan) - 1) * 0.8 + (uniform() - 0.5) * 0.2)
        : randomDirection();
    const auto node = foods[food].node;
    tips.push_back({node, chooseTarget(node, direction), food, direction, 0, 0, true});
}

void PhysarumSimulation::replenishTips() {
    std::vector<unsigned> counts(foods.size());
    for (const auto& tip : tips) ++counts[tip.food];
    for (std::size_t i = 0; i < foods.size() && tips.size() < settings.maxTips; ++i) {
        const auto food = nextFood;
        nextFood = (nextFood + 1) % foods.size();
        if (counts[food] == 0) { launchFoodTip(food); ++counts[food]; }
    }
}

std::size_t PhysarumSimulation::chooseTarget(std::size_t node, Position direction, std::size_t avoid) const {
    std::size_t selected = noNode;
    double best = -1e30;
    const auto root = component(node);
    const bool separate = std::any_of(foods.begin(), foods.end(), [&](const Food& food) { return component(food.node) != root; });
    for (std::size_t i = 0; i < foods.size(); ++i) {
        if ((separate && component(foods[i].node) == root) || distance(nodes[node].position, foods[i].position) < 2 * maxStep) continue;
        const auto delta = subtract(foods[i].position, nodes[node].position);
        const double score = dot(direction, unit(delta)) + 0.1 / (0.1 + distance(nodes[node].position, foods[i].position)) -
                             (i == avoid ? 0.4 : 0);
        if (score > best) { best = score; selected = i; }
    }
    return selected;
}

bool PhysarumSimulation::addEdge(std::size_t a, std::size_t b) {
    const double length = distance(nodes[a].position, nodes[b].position);
    if (a == b || length < 1e-7 || length > maxStep + 1e-12) return false;
    for (const auto& edge : edges) if (edge.active &&
        ((edge.a == a && edge.b == b) || (edge.a == b && edge.b == a))) return false;
    edges.push_back({a, b, length, 0.003, 0, iteration, true});
    edges.back().state = TubeState::Exploring;
    const auto rootA = component(a), rootB = component(b);
    parents[std::max(rootA, rootB)] = std::min(rootA, rootB);
    return true;
}

bool PhysarumSimulation::extend(Tip& tip) {
    const auto from = nodes[tip.node].position;
    std::vector<std::size_t> neighbors;
    for (const auto& edge : edges) if (edge.active) {
        if (edge.a == tip.node) neighbors.push_back(edge.b);
        if (edge.b == tip.node) neighbors.push_back(edge.a);
    }
    const auto adjacent = [&](std::size_t node) {
        return std::find(neighbors.begin(), neighbors.end(), node) != neighbors.end();
    };
    std::size_t food = noNode, destination = noNode;
    Position next;
    // Food approaches use the same collision checks as ordinary growth, so a
    // tube encountered before the food also becomes a hydraulic junction.
    for (std::size_t i = 0; i < foods.size(); ++i) {
        const double d = distance(from, foods[i].position);
        if (d > 1e-7 && d <= maxStep && !adjacent(foods[i].node)) {
            food = i; destination = foods[i].node; next = foods[i].position;
            break;
        }
    }
    if (tip.target == noNode || distance(from, foods[tip.target].position) < 2 * maxStep)
        tip.target = chooseTarget(tip.node, tip.direction);
    // Persistent segments, stochastic bifurcations, and a modest nutrient bias
    // supply growth; hydraulic adaptation subsequently selects the useful tubes.
    if (food == noNode) {
        if (tip.age % 4 == 0) {
            tip.direction = rotate(tip.direction, (uniform() - 0.5) * 0.3);
            if (tip.target != noNode) {
                const auto target = unit(subtract(foods[tip.target].position, from));
                const double bias = settings.foodBias * (component(tip.node) == component(foods[tip.target].node) ? 0.35 : 1.0);
                tip.direction = unit(add(scale(tip.direction, 1 - bias), scale(target, bias)));
            }
        }
        const double length = maxStep * (0.82 + 0.18 * uniform());
        next = add(from, scale(tip.direction, length));
        if (!inside(next)) {
            tip.direction = rotate(tip.direction, uniform() < 0.5 ? -1.1 : 1.1);
            next = add(from, scale(tip.direction, length));
            if (!inside(next)) return false;
        }
        // Snap first, then test the actual connecting segment for crossings.
        // Otherwise the snap itself could pass through a tube without joining it.
        double nearest = 0.0035;
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (i == tip.node || adjacent(i)) continue;
            const double d = distance(next, nodes[i].position);
            if (d < nearest && distance(from, nodes[i].position) <= maxStep) {
                nearest = d; destination = i;
            }
        }
        if (destination != noNode) next = nodes[destination].position;
    }
    // Anastomosis: a crossing becomes a genuine hydraulic junction. Split the
    // existing tube, preserving its conductivity, rather than drawing a crossing.
    std::size_t hit = noNode;
    double first = 2;
    const auto ray = subtract(next, from);
    for (std::size_t e = 0; e < edges.size(); ++e) {
        const auto& edge = edges[e];
        if (!edge.active || edge.a == tip.node || edge.b == tip.node) continue;
        const auto a = nodes[edge.a].position;
        const auto segment = subtract(nodes[edge.b].position, a);
        const double denominator = cross(ray, segment);
        double t, u = 0;
        if (std::abs(denominator) < 1e-12) {
            if (std::abs(cross(subtract(a, from), ray)) > 1e-12) continue;
            const double length2 = dot(ray, ray);
            t = std::min(dot(subtract(a, from), ray), dot(subtract(nodes[edge.b].position, from), ray)) / length2;
        } else {
            t = cross(subtract(a, from), segment) / denominator;
            u = cross(subtract(a, from), ray) / denominator;
        }
        if (t > 1e-4 && t <= 1 && u >= 0 && u <= 1 && t < first) { first = t; hit = e; }
    }
    const bool joining = hit != noNode || destination != noNode;
    if (hit != noNode) {
        const auto intersection = add(from, scale(ray, first));
        const auto old = edges[hit];
        if (distance(intersection, nodes[old.a].position) < 1e-6 && distance(from, nodes[old.a].position) <= maxStep)
            destination = old.a;
        else if (distance(intersection, nodes[old.b].position) < 1e-6 && distance(from, nodes[old.b].position) <= maxStep)
            destination = old.b;
        else {
            destination = addNode(intersection, nodes[tip.node].pressure);
            parents[destination] = component(old.a);
            edges[hit].b = destination;
            edges[hit].length = distance(nodes[old.a].position, intersection);
            auto tail = old;
            tail.a = destination; tail.length = distance(intersection, nodes[old.b].position);
            edges.push_back(tail);
        }
    }
    if (destination == noNode) {
        destination = addNode(next, nodes[tip.node].pressure);
        if (!addEdge(tip.node, destination)) { nodes.pop_back(); parents.pop_back(); return false; }
    } else if (!addEdge(tip.node, destination)) return false;
    tip.node = destination;
    // Fusion can finish this frontier, but never deletes either side of the
    // resulting loop. Both routes compete through subsequent hydraulic solves.
    if (joining) {
        tip.active = false;
        // Every frontier at the contact point is consumed. A frontier beyond
        // an interior branch contact remains active on its distal portion.
        for (auto& other : tips) if (other.node == destination) other.active = false;
    }
    return true;
}

void PhysarumSimulation::grow() {
    // A full live-node budget pauses expansion, not the lifetime of its tips.
    // Pruning can release capacity and let surviving frontiers resume later.
    if (nodes.size() >= settings.maxNodes) return;
    const std::size_t count = tips.size();
    for (std::size_t i = 0; i < count && nodes.size() < settings.maxNodes; ++i) {
        auto& tip = tips[i];
        if (!tip.active) continue;
        if (!extend(tip)) { tip.active = false; continue; }
        ++tip.age; ++tip.sinceBranch;
        if (tip.active && tip.sinceBranch >= 6 && tips.size() < settings.maxTips && uniform() < settings.branching) {
            const double angle = (uniform() < 0.5 ? -1 : 1) * (0.55 + 0.35 * uniform());
            auto direction = rotate(tip.direction, angle);
            const auto target = chooseTarget(tip.node, direction, tip.target);
            tip.sinceBranch = 0;
            tips.push_back({tip.node, target, tip.food, direction, 0, 0, true});
        }
    }
    tips.erase(std::remove_if(tips.begin(), tips.end(), [](const Tip& tip) { return !tip.active; }), tips.end());
    // Nourished trunks can launch lateral probes between existing food routes.
    if (tips.size() < settings.maxTips && !edges.empty() && uniform() < settings.branching) {
        for (unsigned attempt = 0; attempt < 8; ++attempt) {
            const auto& edge = edges[std::size_t(uniform() * edges.size())];
            const auto node = uniform() < 0.5 ? edge.a : edge.b;
            if (edge.foodLoad < 0.01 || iteration - nodes[node].born < 40 ||
                std::any_of(tips.begin(), tips.end(), [&](const Tip& tip) { return distance(nodes[tip.node].position, nodes[node].position) < 0.03; })) continue;
            std::size_t food = noNode;
            double nearest = 2;
            for (std::size_t f = 0; f < foods.size(); ++f) if (component(foods[f].node) == component(node)) {
                const double d = distance(nodes[node].position, foods[f].position);
                if (d < nearest) { food = f; nearest = d; }
            }
            const auto direction = rotate(unit(subtract(nodes[edge.b].position, nodes[edge.a].position)), uniform() < 0.5 ? -1.1 : 1.1);
            tips.push_back({node, chooseTarget(node, direction), food, direction, 0, 0, true});
            break;
        }
    }
    if (nodes.size() < settings.maxNodes) replenishTips();
}

void PhysarumSimulation::updateExploration() {
    std::vector<std::vector<std::size_t>> incident(nodes.size());
    std::vector<unsigned> degree(nodes.size());
    std::vector<bool> anchors(nodes.size()), peeled(edges.size());
    for (const auto& food : foods) anchors[food.node] = true;
    for (std::size_t e = 0; e < edges.size(); ++e) if (edges[e].active) {
        const auto& edge = edges[e];
        incident[edge.a].push_back(e); incident[edge.b].push_back(e);
        ++degree[edge.a]; ++degree[edge.b];
        if (edge.state == TubeState::Established) anchors[edge.a] = anchors[edge.b] = true;
    }
    // Strip pendant branches. What remains joins food/trunk anchors or closes
    // a loop, so its newly connected segments now need hydraulic reinforcement.
    std::queue<std::size_t> queue;
    for (std::size_t i = 0; i < nodes.size(); ++i) if (!anchors[i] && degree[i] <= 1) queue.push(i);
    while (!queue.empty()) {
        const auto node = queue.front(); queue.pop();
        for (auto e : incident[node]) if (!peeled[e]) {
            peeled[e] = true;
            const auto& edge = edges[e];
            const auto other = edge.a == node ? edge.b : edge.a;
            --degree[node]; --degree[other];
            if (!anchors[other] && degree[other] == 1) queue.push(other);
        }
    }
    for (std::size_t e = 0; e < edges.size(); ++e) if (edges[e].active && !peeled[e]) edges[e].state = TubeState::Established;
    for (const auto& edge : edges) if (edge.active && edge.state == TubeState::Established)
        anchors[edge.a] = anchors[edge.b] = true;

    // Orient the remaining exploratory trees away from their anchors. Protect
    // only the union of paths to surviving tips, including shared fork stems.
    std::vector<std::size_t> parentEdge(nodes.size(), noNode);
    std::vector<bool> seen = anchors, protectedEdge(edges.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) if (anchors[i]) queue.push(i);
    while (!queue.empty()) {
        const auto node = queue.front(); queue.pop();
        for (auto e : incident[node]) if (edges[e].state != TubeState::Established) {
            const auto& edge = edges[e];
            const auto other = edge.a == node ? edge.b : edge.a;
            if (seen[other]) continue;
            seen[other] = true; parentEdge[other] = e; queue.push(other);
        }
    }
    for (const auto& tip : tips) if (tip.active) {
        auto node = tip.node;
        while (parentEdge[node] != noNode) {
            const auto e = parentEdge[node];
            if (protectedEdge[e]) break;
            protectedEdge[e] = true;
            node = edges[e].a == node ? edges[e].b : edges[e].a;
        }
    }
    for (std::size_t e = 0; e < edges.size(); ++e) if (edges[e].active && edges[e].state != TubeState::Established)
        edges[e].state = protectedEdge[e] ? TubeState::Exploring : TubeState::Abandoned;
}

void PhysarumSimulation::adaptTubes(std::vector<Edge>& tubes, const Settings& settings, unsigned iteration) {
    for (auto& edge : tubes) if (edge.active && edge.born < iteration) {
        if (edge.state == TubeState::Exploring) { edge.weakSteps = 0; continue; }
        const double target = settings.reinforcement * edge.foodLoad;
        edge.conductivity = std::clamp(edge.conductivity + settings.adaptation * (target - edge.conductivity), 1e-6, 3.0);
        const bool weak = edge.conductivity < 0.002 && edge.foodLoad < 0.002 &&
                          target <= edge.conductivity;
        edge.weakSteps = weak ? edge.weakSteps + 1 : 0;
        if (edge.weakSteps >= pruningDelay) edge.active = false;
    }
}

void PhysarumSimulation::retire() {
    rebuildComponents();
    std::vector<bool> supplied(nodes.size());
    for (const auto& food : foods) supplied[component(food.node)] = true;
    std::vector<bool> alive(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) alive[i] = supplied[component(i)];
    std::vector<std::size_t> remap(nodes.size(), noNode);
    std::vector<Node> liveNodes;
    std::vector<Edge> liveEdges;
    liveNodes.reserve(nodes.size());
    liveEdges.reserve(edges.size());
    // Every food is permanent. Fragments disconnected from all foods retire.
    for (std::size_t i = 0; i < nodes.size(); ++i) if (alive[i]) {
        remap[i] = liveNodes.size();
        liveNodes.push_back(nodes[i]);
    }
    for (const auto& edge : edges) {
        if (edge.active && alive[edge.a] && alive[edge.b]) {
            auto tube = edge;
            tube.a = remap[edge.a]; tube.b = remap[edge.b];
            liveEdges.push_back(tube);
        } else {
            retired.push_back({nodes[edge.a].position, nodes[edge.b].position});
            if (retired.size() > retiredHistoryLimit) retired.pop_front();
        }
    }
    tips.erase(std::remove_if(tips.begin(), tips.end(), [&](const Tip& tip) {
        return !tip.active || remap[tip.node] == noNode;
    }), tips.end());
    for (auto& tip : tips) tip.node = remap[tip.node];
    for (auto& food : foods) food.node = remap[food.node];
    nodes.swap(liveNodes);
    edges.swap(liveEdges);
    rebuildComponents();
    const auto provider = [&](std::size_t node, std::size_t food) {
        if (component(node) != component(foods[food].node))
            for (std::size_t f = 0; f < foods.size(); ++f) if (component(node) == component(foods[f].node)) return f;
        return food;
    };
    for (auto& tip : tips) tip.food = provider(tip.node, tip.food);
}

void PhysarumSimulation::transport() {
    std::vector<std::vector<std::size_t>> groups(nodes.size());
    for (const auto& food : foods) groups[component(food.node)].push_back(food.node);
    std::vector<double> supply(nodes.size()), pumpingPressure(nodes.size());
    for (auto& edge : edges) edge.foodLoad = 0;
    // Protected trees have no sinks and carry exactly zero flow. Leaving them
    // out of the solve also avoids conditioning the mesh on thousands of scouts.
    auto hydraulic = edges;
    for (auto& edge : hydraulic) if (edge.state != TubeState::Established) edge.active = false;
    injections.assign(nodes.size(), 0);
    // Each independent colony balances its own flow. Root-to-food basis solves
    // span every food pair within that colony; no flow crosses an unjoined gap.
    for (std::size_t root = 0; root < groups.size(); ++root) {
        const auto& ports = groups[root];
        if (ports.size() < 2) continue;
        std::vector<std::vector<double>> basis(ports.size(), std::vector<double>(nodes.size()));
        for (std::size_t k = 1; k < ports.size(); ++k) {
            std::fill(supply.begin(), supply.end(), 0);
            supply[ports[0]] = 1; supply[ports[k]] = -1;
            solveFlow(nodes, hydraulic, supply);
            for (std::size_t i = 0; i < nodes.size(); ++i) basis[k][i] = nodes[i].pressure;
        }
        const std::size_t pairs = ports.size() * (ports.size() - 1) / 2;
        const std::size_t selected = (iteration / 16) % pairs;
        std::size_t source = 0, sink = 0, pair = 0;
        for (std::size_t a = 0; a < ports.size(); ++a) for (std::size_t b = a + 1; b < ports.size(); ++b, ++pair) {
            if (pair == selected) { source = a; sink = b; }
            for (auto& edge : edges) if (edge.state == TubeState::Established && component(edge.a) == root) {
                const double flow = edge.conductivity / edge.length *
                    ((basis[b][edge.a] - basis[a][edge.a]) - (basis[b][edge.b] - basis[a][edge.b]));
                edge.foodLoad += std::abs(flow) / pairs;
            }
        }
        const double sign = ((iteration / (16 * pairs)) % 2) ? -1.0 : 1.0;
        injections[ports[source]] += sign; injections[ports[sink]] -= sign;
        for (std::size_t i = 0; i < nodes.size(); ++i) if (component(i) == root)
            pumpingPressure[i] = sign * (basis[sink][i] - basis[source][i]);
    }

    for (std::size_t i = 0; i < nodes.size(); ++i) nodes[i].pressure = pumpingPressure[i];
    balanceError = solveFlow(nodes, hydraulic, injections);
    for (std::size_t e = 0; e < edges.size(); ++e) edges[e].flow = hydraulic[e].flow;

    // A pendant tree has its anchor's pressure throughout, consistent with Q=0.
    std::vector<std::vector<std::size_t>> neighbors(nodes.size());
    std::vector<bool> known(nodes.size());
    for (const auto& food : foods) known[food.node] = true;
    for (const auto& edge : edges) if (edge.active) {
        if (edge.state == TubeState::Established) { known[edge.a] = known[edge.b] = true; continue; }
        neighbors[edge.a].push_back(edge.b); neighbors[edge.b].push_back(edge.a);
    }
    std::queue<std::size_t> queue;
    for (std::size_t i = 0; i < nodes.size(); ++i) if (known[i]) queue.push(i);
    while (!queue.empty()) {
        const auto node = queue.front(); queue.pop();
        for (auto other : neighbors[node]) if (!known[other]) {
            nodes[other].pressure = nodes[node].pressure;
            known[other] = true; queue.push(other);
        }
    }
}

double PhysarumSimulation::solveFlow(std::vector<Node>& points, std::vector<Edge>& tubes,
                                     const std::vector<double>& supply) {
    if (points.empty() || supply.size() != points.size()) throw std::invalid_argument("Invalid flow network");
    for (const auto& edge : tubes)
        if (edge.a >= points.size() || edge.b >= points.size() || edge.a == edge.b ||
            !std::isfinite(edge.length) || edge.length <= 0 ||
            !std::isfinite(edge.conductivity) || edge.conductivity <= 0)
            throw std::invalid_argument("Invalid tube geometry or conductivity");
    for (const auto& node : points) if (!std::isfinite(node.pressure))
        throw std::invalid_argument("Node pressure must be finite");
    const auto neighbors = adjacency(points, tubes);
    const auto roots = components(neighbors);
    const std::size_t n = points.size();
    std::vector<std::size_t> unknowns;
    std::vector<double> diagonal(n, 1), x(n), rhs = supply, r(n), z(n), direction(n), product(n);
    std::vector<double> sum(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(supply[i])) throw std::invalid_argument("Flow injections must be finite");
        sum[roots[i]] += supply[i];
        if (roots[i] != i) {
            unknowns.push_back(i);
            diagonal[i] = 0;
            for (const auto& neighbor : neighbors[i]) diagonal[i] += neighbor.second;
            x[i] = points[i].pressure - points[roots[i]].pressure;
        } else rhs[i] = 0;
    }
    for (auto total : sum) if (std::abs(total) > 1e-8)
        throw std::invalid_argument("Flow supply must balance demand within each component");
    auto apply = [&](const std::vector<double>& input, std::vector<double>& output) {
        for (auto i : unknowns) {
            output[i] = diagonal[i] * input[i];
            for (const auto& neighbor : neighbors[i]) if (roots[neighbor.first] != neighbor.first)
                output[i] -= neighbor.second * input[neighbor.first];
        }
    };
    apply(x, product);
    double rz = 0;
    for (auto i : unknowns) {
        r[i] = rhs[i] - product[i];
        z[i] = r[i] / diagonal[i];
        direction[i] = z[i];
        rz += r[i] * z[i];
    }
    // Jacobi-preconditioned conjugate gradients on the grounded graph Laplacian.
    for (std::size_t k = 0; k < std::min<std::size_t>(6 * unknowns.size() + 20, 6000) && rz > 1e-32; ++k) {
        apply(direction, product);
        double denominator = 0;
        for (auto i : unknowns) denominator += direction[i] * product[i];
        if (denominator <= 0) break;
        const double alpha = rz / denominator;
        double next = 0, residual = 0;
        for (auto i : unknowns) {
            x[i] += alpha * direction[i];
            r[i] -= alpha * product[i];
            z[i] = r[i] / diagonal[i];
            next += r[i] * z[i];
            residual += r[i] * r[i];
        }
        if (residual < 1e-20) break;
        for (auto i : unknowns) direction[i] = z[i] + (next / rz) * direction[i];
        rz = next;
    }
    for (std::size_t i = 0; i < n; ++i) points[i].pressure = x[i];
    std::vector<double> balance(n);
    for (auto& edge : tubes) {
        edge.flow = edge.active ?
            edge.conductivity / edge.length * (points[edge.a].pressure - points[edge.b].pressure) : 0;
        balance[edge.a] += edge.flow;
        balance[edge.b] -= edge.flow;
    }
    double error = 0;
    for (std::size_t i = 0; i < n; ++i) error = std::max(error, std::abs(balance[i] - supply[i]));
    return error;
}

bool PhysarumSimulation::step() {
    if (nodes.empty()) return false;
    adaptTubes(edges, settings, iteration);
    retire();
    grow();
    updateExploration();
    transport();
    ++iteration;
    measure();
    return true;
}

void PhysarumSimulation::measure() {
    double closest = history.back();
    const auto a = component(foods[0].node), b = component(foods[1].node);
    if (a == b) closest = 0;
    else if (closest > 0)
        for (std::size_t i = 0; i < nodes.size(); ++i) if (component(i) == a)
            for (std::size_t j = 0; j < nodes.size(); ++j) if (component(j) == b)
                closest = std::min(closest, distance(nodes[i].position, nodes[j].position));
    history.push_back(closest);
    if (history.size() > progressHistoryLimit) history.pop_front();
}
unsigned PhysarumSimulation::getFoodGroups() const {
    std::vector<std::size_t> roots;
    for (const auto& food : foods) roots.push_back(component(food.node));
    std::sort(roots.begin(), roots.end());
    return std::unique(roots.begin(), roots.end()) - roots.begin();
}
unsigned PhysarumSimulation::getActiveEdges() const {
    return std::count_if(edges.begin(), edges.end(), [](const Edge& edge) { return edge.active; });
}
double PhysarumSimulation::getNetworkLength() const {
    double length = 0;
    for (const auto& edge : edges) if (edge.active) length += edge.length;
    return length;
}
double PhysarumSimulation::getTransportLength() const {
    double length = 0;
    for (const auto& edge : edges) if (edge.active && edge.foodLoad >= 0.01 && edge.conductivity >= 0.002)
        length += edge.length;
    return length;
}
double PhysarumSimulation::getConnectionLength(bool transportOnly) const {
    if (foods.size() < 2 || component(foods[0].node) != component(foods[1].node))
        return std::numeric_limits<double>::infinity();
    std::vector<double> lengths(nodes.size(), std::numeric_limits<double>::infinity());
    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    lengths[foods[0].node] = 0; queue.push({0, foods[0].node});
    std::vector<std::vector<std::pair<std::size_t, double>>> neighbors(nodes.size());
    for (const auto& edge : edges) if (edge.active &&
        (!transportOnly || (edge.foodLoad >= 0.01 && edge.conductivity >= 0.002))) {
        neighbors[edge.a].push_back({edge.b, edge.length}); neighbors[edge.b].push_back({edge.a, edge.length});
    }
    while (!queue.empty()) {
        const auto [length, node] = queue.top(); queue.pop();
        if (node == foods[1].node) return length;
        if (length > lengths[node]) continue;
        for (const auto& neighbor : neighbors[node]) if (length + neighbor.second < lengths[neighbor.first]) {
            lengths[neighbor.first] = length + neighbor.second;
            queue.push({lengths[neighbor.first], neighbor.first});
        }
    }
    return std::numeric_limits<double>::infinity();
}
