#include "PhysarumSimulation.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
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
std::vector<bool> reachable(const Adjacency& neighbors) {
    std::vector<bool> seen(neighbors.size());
    if (seen.empty()) return seen;
    std::queue<std::size_t> queue;
    queue.push(0); seen[0] = true;
    while (!queue.empty()) {
        const auto node = queue.front(); queue.pop();
        for (const auto& neighbor : neighbors[node]) if (!seen[neighbor.first]) {
            seen[neighbor.first] = true;
            queue.push(neighbor.first);
        }
    }
    return seen;
}
}

double PhysarumSimulation::distance(Position a, Position b) { return std::hypot(a.x - b.x, a.y - b.y); }
double PhysarumSimulation::uniform() { return std::generate_canonical<double, 53>(random); }
PhysarumSimulation::Position PhysarumSimulation::randomDirection() {
    const double angle = uniform() * 2 * std::acos(-1.0);
    return {std::cos(angle), std::sin(angle)};
}

void PhysarumSimulation::reset(const Settings& s, std::uint32_t seed) {
    if (s.maxNodes < 2 || s.maxNodes > 10000 ||
        s.maxTips == 0 || s.maxTips > 128 || !std::isfinite(s.branching) || s.branching < 0 || s.branching > 1 ||
        !std::isfinite(s.foodBias) || s.foodBias < 0 || s.foodBias > 1 ||
        !std::isfinite(s.adaptation) || s.adaptation <= 0 || s.adaptation > 1 ||
        !std::isfinite(s.reinforcement) || s.reinforcement < 0 || s.reinforcement > 3 ||
        !std::isfinite(s.shortening) || s.shortening < 0 || s.shortening > 1)
        throw std::invalid_argument("Invalid growing-graph settings");
    settings = s;
    random.seed(seed);
    iteration = 0;
    balanceError = 0;
    nodes = {{start, 0, 0, true}};
    edges.clear(); tips.clear(); retired.clear();
    tips.reserve(settings.maxTips);
    foods = {{start, 0}, {goal, noNode}};
    if (settings.sixFoods) {
        for (Position p : {Position{0.25, 0.16}, Position{0.72, 0.18},
                           Position{0.3, 0.82}, Position{0.78, 0.8}}) foods.push_back({p, noNode});
    }
    for (unsigned i = 0; i < std::min(3u, settings.maxTips); ++i) {
        const auto direction = rotate({1, 0}, (int(i) - 1) * 0.8 + (uniform() - 0.5) * 0.2);
        tips.push_back({0, noNode, chooseTarget(0, direction), direction, 0, 0, true});
    }
    injections.assign(1, 0);
    history.assign(1, distance(start, goal));
}

void PhysarumSimulation::setTipLimit(unsigned limit) {
    if (limit == 0 || limit > 128) throw std::invalid_argument("Tip limit must be between 1 and 128");
    settings.maxTips = limit;
    tips.reserve(limit);
    if (tips.size() > limit) tips.resize(limit);
    if (!nodes.empty()) transport();
}

std::size_t PhysarumSimulation::chooseTarget(std::size_t node, Position direction, std::size_t avoid) const {
    std::size_t selected = noNode;
    double best = -1e30;
    for (std::size_t i = 0; i < foods.size(); ++i) if (foods[i].node == noNode) {
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
    return true;
}

bool PhysarumSimulation::extend(Tip& tip) {
    const auto from = nodes[tip.node].position;
    // Food is attached only when a growing tip physically reaches it.
    for (std::size_t i = 0; i < foods.size(); ++i) {
        const double d = distance(from, foods[i].position);
        if (d > 1e-7 && d <= maxStep) {
            const bool discovered = foods[i].node == noNode;
            std::size_t destination = foods[i].node;
            if (discovered) {
                destination = nodes.size();
                nodes.push_back({foods[i].position, nodes[tip.node].pressure, iteration, true});
            }
            if (addEdge(tip.node, destination)) {
                foods[i].node = destination;
                tip.previous = tip.node; tip.node = destination;
                tip.target = chooseTarget(tip.node, tip.direction);
                if (!discovered) tip.active = false;
                return true;
            }
            if (discovered) nodes.pop_back();
        }
    }
    if (tip.target == noNode || foods[tip.target].node != noNode)
        tip.target = chooseTarget(tip.node, tip.direction);
    // Persistent segments, stochastic bifurcations, and a modest nutrient bias
    // supply growth; hydraulic adaptation subsequently selects the useful tubes.
    if (tip.age % 4 == 0) {
        tip.direction = rotate(tip.direction, (uniform() - 0.5) * 0.3);
        if (tip.target != noNode) {
            const auto target = unit(subtract(foods[tip.target].position, from));
            tip.direction = unit(add(scale(tip.direction, 1 - settings.foodBias), scale(target, settings.foodBias)));
        }
    }
    const double length = maxStep * (0.82 + 0.18 * uniform());
    auto next = add(from, scale(tip.direction, length));
    if (!inside(next)) {
        tip.direction = rotate(tip.direction, uniform() < 0.5 ? -1.1 : 1.1);
        next = add(from, scale(tip.direction, length));
        if (!inside(next)) return false;
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
        if (std::abs(denominator) < 1e-12) continue;
        const double t = cross(subtract(a, from), segment) / denominator;
        const double u = cross(subtract(a, from), ray) / denominator;
        if (t > 1e-4 && t <= 1 && u >= 0 && u <= 1 && t < first) { first = t; hit = e; }
    }
    if (hit != noNode) {
        const auto intersection = add(from, scale(ray, first));
        const auto old = edges[hit];
        std::size_t junction;
        if (distance(intersection, nodes[old.a].position) < 1e-6 && distance(from, nodes[old.a].position) <= maxStep)
            junction = old.a;
        else if (distance(intersection, nodes[old.b].position) < 1e-6 && distance(from, nodes[old.b].position) <= maxStep)
            junction = old.b;
        else {
            junction = nodes.size();
            nodes.push_back({intersection, nodes[tip.node].pressure, iteration, true});
            edges[hit].b = junction;
            edges[hit].length = distance(nodes[old.a].position, intersection);
            auto tail = old;
            tail.a = junction; tail.length = distance(intersection, nodes[old.b].position);
            edges.push_back(tail);
        }
        if (!addEdge(tip.node, junction)) return false;
        tip.previous = tip.node;
        tip.node = junction;
        tip.active = tip.target != noNode && foods[tip.target].node == noNode;
        return true;
    }
    // Join a nearby existing node only if the full connecting edge is local.
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (i == tip.node || i == tip.previous) continue;
        if (distance(next, nodes[i].position) < 0.0035 && distance(from, nodes[i].position) <= maxStep) {
            if (addEdge(tip.node, i)) {
                tip.previous = tip.node; tip.node = i;
                tip.active = tip.target != noNode && foods[tip.target].node == noNode;
                return true;
            }
            return false;
        }
    }
    const auto node = nodes.size();
    nodes.push_back({next, nodes[tip.node].pressure, iteration, true});
    if (!addEdge(tip.node, node)) { nodes.pop_back(); return false; }
    tip.previous = tip.node; tip.node = node;
    return true;
}

void PhysarumSimulation::grow() {
    // A full live-node budget pauses expansion, not the lifetime of its tips.
    // Pruning can release capacity and let surviving frontiers resume later.
    if (nodes.size() >= settings.maxNodes) return;
    const std::size_t count = tips.size();
    for (std::size_t i = 0; i < count && nodes.size() < settings.maxNodes; ++i) {
        auto& tip = tips[i];
        if (!tip.active || !nodes[tip.node].connected) { tip.active = false; continue; }
        if (!extend(tip)) { tip.active = false; continue; }
        ++tip.age; ++tip.sinceBranch;
        if (tip.active && tip.sinceBranch >= 6 && tips.size() < settings.maxTips && uniform() < settings.branching) {
            const double angle = (uniform() < 0.5 ? -1 : 1) * (0.55 + 0.35 * uniform());
            auto direction = rotate(tip.direction, angle);
            const auto target = chooseTarget(tip.node, direction, tip.target);
            tip.sinceBranch = 0;
            tips.push_back({tip.node, tip.previous, target, direction, 0, 0, true});
        }
    }
    tips.erase(std::remove_if(tips.begin(), tips.end(), [](const Tip& tip) { return !tip.active; }), tips.end());
    if (tips.empty() && nodes.size() < settings.maxNodes) {
        // New growth can sprout from already explored, nourished sites; no
        // disconnected nodes are introduced to seed remote exploration.
        for (const auto& food : foods) if (food.node != noNode && nodes[food.node].connected) {
            auto direction = randomDirection();
            tips.push_back({food.node, noNode, chooseTarget(food.node, direction), direction, 0, 0, true});
            if (tips.size() >= settings.maxTips) break;
        }
    }
}

void PhysarumSimulation::adapt() {
    for (auto& edge : edges) if (edge.active && edge.born < iteration) {
        // Food-to-food transport selects the backbone. Provisional frontier
        // demand can support only a thin foraging tube, never a replacement trunk.
        const double target = settings.reinforcement * edge.foodDrive + edge.explorationDrive;
        edge.conductivity = std::clamp(edge.conductivity + settings.adaptation * (target - edge.conductivity), 1e-6, 3.0);
        if (iteration - edge.born > 30 && edge.conductivity < 0.002 &&
            edge.foodLoad < 0.002 && edge.explorationDrive < 0.001 && target <= edge.conductivity)
            edge.active = false;
    }
}

double PhysarumSimulation::flowResponse(double flow, bool multipleFoods) {
    const double magnitude = std::abs(flow);
    if (!multipleFoods) return magnitude; // Linear TKN response: shortest-path selection.
    const double power = std::pow(magnitude, 1.8);
    return 2 * power / (1 + power); // Saturating multi-food response, normalized f(1)=1.
}

void PhysarumSimulation::retire() {
    const auto alive = reachable(adjacency(nodes, edges));
    std::vector<std::size_t> remap(nodes.size(), noNode);
    std::vector<Node> liveNodes;
    std::vector<Edge> liveEdges;
    liveNodes.reserve(nodes.size());
    liveEdges.reserve(edges.size());
    // The reference seed is always retained, even when every tube has died.
    // All other nodes need a live connection to it to consume node capacity.
    for (std::size_t i = 0; i < nodes.size(); ++i) if (alive[i]) {
        remap[i] = liveNodes.size();
        liveNodes.push_back(nodes[i]);
        liveNodes.back().connected = true;
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
    for (auto& tip : tips) {
        tip.node = remap[tip.node];
        if (tip.previous != noNode) tip.previous = remap[tip.previous];
    }
    for (auto& food : foods) if (food.node != noNode) food.node = remap[food.node];
    nodes.swap(liveNodes);
    edges.swap(liveEdges);
}

void PhysarumSimulation::shorten() {
    if (settings.shortening == 0 || edges.empty()) return;
    std::vector<std::vector<std::size_t>> incident(nodes.size());
    for (std::size_t e = 0; e < edges.size(); ++e) {
        incident[edges[e].a].push_back(e); incident[edges[e].b].push_back(e);
    }
    std::vector<bool> fixed(nodes.size());
    fixed[0] = true;
    for (const auto& food : foods) if (food.node != noNode) fixed[food.node] = true;
    for (const auto& tip : tips) fixed[tip.node] = true;

    // Broad-phase collision bins include the maximum body motion across all six
    // relaxation sweeps. Shortening cannot create an unmodelled crossing/junction.
    constexpr int resolution = 64;
    constexpr double maxMotion = 0.0005;
    constexpr double padding = 6 * maxMotion + 1e-6;
    std::vector<std::vector<std::size_t>> bins(resolution * resolution);
    const auto grid = [](double value) { return std::clamp(int(std::floor(value * resolution)), 0, resolution - 1); };
    for (std::size_t e = 0; e < edges.size(); ++e) {
        const auto a = nodes[edges[e].a].position, b = nodes[edges[e].b].position;
        for (int y = grid(std::min(a.y, b.y) - padding); y <= grid(std::max(a.y, b.y) + padding); ++y)
            for (int x = grid(std::min(a.x, b.x) - padding); x <= grid(std::max(a.x, b.x) + padding); ++x)
                bins[y * resolution + x].push_back(e);
    }
    std::vector<unsigned> checked(edges.size());
    unsigned stamp = 0;
    const auto collides = [&](std::size_t node, std::size_t neighbor, Position a, Position b) {
        ++stamp;
        const auto r = subtract(b, a);
        for (int y = grid(std::min(a.y, b.y)); y <= grid(std::max(a.y, b.y)); ++y)
            for (int x = grid(std::min(a.x, b.x)); x <= grid(std::max(a.x, b.x)); ++x)
                for (auto e : bins[y * resolution + x]) {
                    if (checked[e] == stamp) continue;
                    checked[e] = stamp;
                    const auto& other = edges[e];
                    if (other.a == node || other.b == node || other.a == neighbor || other.b == neighbor) continue;
                    const auto c = nodes[other.a].position, d = nodes[other.b].position;
                    const auto v = subtract(d, c);
                    const double denominator = cross(r, v);
                    if (std::abs(denominator) > 1e-14) {
                        const double t = cross(subtract(c, a), v) / denominator;
                        const double u = cross(subtract(c, a), r) / denominator;
                        if (t >= -1e-9 && t <= 1 + 1e-9 && u >= -1e-9 && u <= 1 + 1e-9) return true;
                    } else if (std::abs(cross(subtract(c, a), r)) < 1e-12) {
                        const double length2 = dot(r, r);
                        const double t0 = dot(subtract(c, a), r) / length2;
                        const double t1 = dot(subtract(d, a), r) / length2;
                        if (std::min(1.0, std::max(t0, t1)) > std::max(0.0, std::min(t0, t1)) + 1e-9) return true;
                    }
                }
        return false;
    };
    for (int sweep = 0; sweep < 6; ++sweep) for (std::size_t i = 1; i < nodes.size(); ++i) {
        if (fixed[i] || iteration - nodes[i].born < 40 || incident[i].size() < 2) continue;
        const auto before = nodes[i].position;
        Position force{};
        double weightSum = 0, energy = 0;
        for (auto e : incident[i]) {
            const auto& edge = edges[e];
            const double weight = std::sqrt(edge.foodLoad);
            const auto neighbor = edge.a == i ? edge.b : edge.a;
            force = add(force, scale(subtract(nodes[neighbor].position, before), weight / edge.length));
            weightSum += weight;
            energy += weight * edge.length;
        }
        if (weightSum < 0.01) continue;
        auto move = scale(force, settings.shortening * maxStep / weightSum);
        const double length = std::hypot(move.x, move.y);
        if (length < 1e-9) continue;
        if (length > maxMotion) move = scale(move, maxMotion / length);
        for (int attempt = 0; attempt < 6; ++attempt) {
            const auto candidate = add(before, move);
            bool valid = inside(candidate);
            double proposed = 0;
            for (auto e : incident[i]) {
                const auto& edge = edges[e];
                const auto neighbor = edge.a == i ? edge.b : edge.a;
                const double nextLength = distance(candidate, nodes[neighbor].position);
                if (nextLength > maxStep + 1e-12 || nextLength < std::min(edge.length, 0.0001)) { valid = false; break; }
                proposed += std::sqrt(edge.foodLoad) * nextLength;
            }
            if (valid && proposed < energy - 1e-12) {
                for (auto e : incident[i]) {
                    const auto& edge = edges[e];
                    const auto neighbor = edge.a == i ? edge.b : edge.a;
                    if (collides(i, neighbor, candidate, nodes[neighbor].position)) { valid = false; break; }
                }
                if (valid) {
                    nodes[i].position = candidate;
                    for (auto e : incident[i]) edges[e].length = distance(nodes[edges[e].a].position, nodes[edges[e].b].position);
                    break;
                }
            }
            move = scale(move, 0.5);
        }
    }
}

void PhysarumSimulation::transport() {
    const auto connected = reachable(adjacency(nodes, edges));
    for (std::size_t i = 0; i < nodes.size(); ++i) nodes[i].connected = connected[i];
    tips.erase(std::remove_if(tips.begin(), tips.end(), [&](const Tip& tip) { return !connected[tip.node]; }), tips.end());
    std::vector<std::size_t> ports, growing;
    for (const auto& food : foods) if (food.node != noNode && connected[food.node]) ports.push_back(food.node);
    const std::size_t unknown = foods.size() - ports.size();
    // Exploration has a finite nutritional purpose. Once every food is joined,
    // scouts can still grow and try shortcuts, but receive no fictitious demand.
    if (unknown > 0 && nodes.size() < settings.maxNodes)
        for (const auto& tip : tips) if (connected[tip.node] &&
            std::find(ports.begin(), ports.end(), tip.node) == ports.end()) growing.push_back(tip.node);

    // M-1 root-to-food pressure solves span every pair's flow by superposition.
    // Average *responses* across all food pairs, rather than rewarding whichever
    // food happens to be pumping this frame or the locations of newborn tips.
    std::vector<std::vector<double>> basis(ports.size(), std::vector<double>(nodes.size()));
    std::vector<double> supply(nodes.size());
    for (std::size_t k = 1; k < ports.size(); ++k) {
        std::fill(supply.begin(), supply.end(), 0);
        supply[ports[0]] = 1; supply[ports[k]] = -1;
        solveFlow(nodes, edges, supply);
        for (std::size_t i = 0; i < nodes.size(); ++i) basis[k][i] = nodes[i].pressure;
    }
    const std::size_t pairs = ports.size() * (ports.size() - 1) / 2;
    std::size_t source = 0, sink = 0, pair = 0;
    const std::size_t selected = pairs ? (iteration / 16) % pairs : 0;
    for (auto& edge : edges) { edge.foodLoad = edge.foodDrive = edge.explorationDrive = 0; }
    for (std::size_t a = 0; a < ports.size(); ++a) for (std::size_t b = a + 1; b < ports.size(); ++b, ++pair) {
        if (pair == selected) { source = a; sink = b; }
        for (auto& edge : edges) {
            const double flow = edge.conductivity / edge.length *
                ((basis[b][edge.a] - basis[a][edge.a]) - (basis[b][edge.b] - basis[a][edge.b]));
            edge.foodLoad += std::abs(flow) / pairs;
            edge.foodDrive += flowResponse(flow, ports.size() > 2) / pairs;
        }
    }
    const double sign = pairs && ((iteration / (16 * pairs)) % 2) ? -1.0 : 1.0;
    injections.assign(nodes.size(), 0);
    if (pairs) {
        injections[ports[source]] = sign; injections[ports[sink]] = -sign;
    }
    if (!growing.empty()) {
        const double growthFlow = 0.12 * unknown / std::max<std::size_t>(1, foods.size() - 1);
        for (auto node : growing) injections[node] -= growthFlow / growing.size();
        for (auto node : ports) injections[node] += growthFlow / ports.size();
    }
    for (std::size_t i = 0; i < nodes.size(); ++i)
        nodes[i].pressure = pairs ? sign * (basis[sink][i] - basis[source][i]) : 0;
    balanceError = solveFlow(nodes, edges, injections);
    if (!growing.empty()) for (auto& edge : edges) {
        const double foodFlow = pairs ? sign * edge.conductivity / edge.length *
            ((basis[sink][edge.a] - basis[source][edge.a]) - (basis[sink][edge.b] - basis[source][edge.b])) : 0;
        edge.explorationDrive = std::min(0.01, std::abs(edge.flow - foodFlow));
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
    const auto connected = reachable(neighbors);
    const std::size_t n = points.size();
    std::vector<double> diagonal(n, 1), x(n), rhs = supply, r(n), z(n), direction(n), product(n);
    double sum = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(supply[i]) || (!connected[i] && std::abs(supply[i]) > 1e-12))
            throw std::invalid_argument("Flow injections must be finite and connected to the reference node");
        sum += supply[i];
        points[i].connected = connected[i];
        if (connected[i] && i != 0) {
            diagonal[i] = 0;
            for (const auto& neighbor : neighbors[i]) diagonal[i] += neighbor.second;
            x[i] = points[i].pressure - points[0].pressure;
        } else rhs[i] = 0;
    }
    if (std::abs(sum) > 1e-8) throw std::invalid_argument("Flow supply must balance demand");
    auto apply = [&](const std::vector<double>& input, std::vector<double>& output) {
        std::fill(output.begin(), output.end(), 0);
        for (std::size_t i = 1; i < n; ++i) if (connected[i]) {
            output[i] = diagonal[i] * input[i];
            for (const auto& neighbor : neighbors[i]) if (neighbor.first != 0)
                output[i] -= neighbor.second * input[neighbor.first];
        }
    };
    apply(x, product);
    double rz = 0;
    for (std::size_t i = 1; i < n; ++i) if (connected[i]) {
        r[i] = rhs[i] - product[i];
        z[i] = r[i] / diagonal[i];
        direction[i] = z[i];
        rz += r[i] * z[i];
    }
    // Jacobi-preconditioned conjugate gradients on the grounded graph Laplacian.
    for (std::size_t k = 0; k < std::min<std::size_t>(6 * n + 20, 6000) && rz > 1e-32; ++k) {
        apply(direction, product);
        const double denominator = std::inner_product(direction.begin(), direction.end(), product.begin(), 0.0);
        if (denominator <= 0) break;
        const double alpha = rz / denominator;
        double next = 0, residual = 0;
        for (std::size_t i = 1; i < n; ++i) if (connected[i]) {
            x[i] += alpha * direction[i];
            r[i] -= alpha * product[i];
            z[i] = r[i] / diagonal[i];
            next += r[i] * z[i];
            residual += r[i] * r[i];
        }
        if (residual < 1e-20) break;
        for (std::size_t i = 1; i < n; ++i) direction[i] = z[i] + (next / rz) * direction[i];
        rz = next;
    }
    for (std::size_t i = 0; i < n; ++i) points[i].pressure = connected[i] ? x[i] : 0;
    std::vector<double> balance(n);
    for (auto& edge : tubes) {
        edge.flow = edge.active && connected[edge.a] && connected[edge.b] ?
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
    adapt();
    retire();
    grow();
    shorten();
    transport();
    ++iteration;
    measure();
    return true;
}

void PhysarumSimulation::measure() {
    double closest = history.back();
    for (const auto& node : nodes) closest = std::min(closest, distance(node.position, goal));
    history.push_back(closest);
    if (history.size() > progressHistoryLimit) history.pop_front();
}
unsigned PhysarumSimulation::getReachedFoods() const {
    return std::count_if(foods.begin(), foods.end(), [](const Food& food) { return food.node != noNode; });
}
unsigned PhysarumSimulation::getActiveEdges() const {
    return std::count_if(edges.begin(), edges.end(), [](const Edge& edge) { return edge.active; });
}
double PhysarumSimulation::getNetworkLength() const {
    double length = 0;
    for (const auto& edge : edges) if (edge.active && nodes[edge.a].connected) length += edge.length;
    return length;
}
double PhysarumSimulation::getTransportLength() const {
    double length = 0;
    for (const auto& edge : edges) if (edge.active && edge.foodLoad >= 0.01 && edge.conductivity >= 0.002)
        length += edge.length;
    return length;
}
double PhysarumSimulation::getConnectionLength(bool transportOnly) const {
    if (foods.size() < 2 || foods[1].node == noNode || !nodes[foods[1].node].connected)
        return std::numeric_limits<double>::infinity();
    std::vector<double> lengths(nodes.size(), std::numeric_limits<double>::infinity());
    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    lengths[0] = 0; queue.push({0, 0});
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
