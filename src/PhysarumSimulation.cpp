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
bool trunksConflict(Position a, Position b, Position c, Position d) {
    const auto ab = subtract(b, a), cd = subtract(d, c);
    const double denominator = cross(ab, cd);
    if (std::abs(denominator) > 1e-14) {
        const double t = cross(subtract(c, a), cd) / denominator;
        const double u = cross(subtract(c, a), ab) / denominator;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return true;
    }
    const auto squaredDistance = [](Position p, Position start, Position ray) {
        const auto delta = subtract(p, start);
        const double length2 = dot(ray, ray);
        const double t = length2 > 0 ? std::clamp(dot(delta, ray) / length2, 0.0, 1.0) : 0;
        const auto offset = subtract(delta, scale(ray, t));
        return dot(offset, offset);
    };
    const double radius = PhysarumSimulation::contactRadius;
    return std::min({squaredDistance(a, c, cd), squaredDistance(b, c, cd),
                     squaredDistance(c, a, ab), squaredDistance(d, a, ab)}) <= radius * radius + 1e-16;
}
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
    if (s.maxNodes < 3 * slimeCount || s.maxNodes > 10000 ||
        s.maxTips == 0 || s.maxTips > 128 || !std::isfinite(s.spawnRate) || s.spawnRate < 0 || s.spawnRate > 4 ||
        !std::isfinite(s.foodBias) || s.foodBias < 0 || s.foodBias > 1 ||
        !std::isfinite(s.adaptation) || s.adaptation <= 0 || s.adaptation > 1 ||
        !std::isfinite(s.reinforcement) || s.reinforcement < 0 || s.reinforcement > 3)
        throw std::invalid_argument("Invalid growing-graph settings");
    settings = s;
    random.seed(seed);
    foods.clear();
    for (unsigned slime = 0; slime < slimeCount; ++slime) {
        const unsigned count = uniform() < 0.5 ? 2 : 3;
        for (unsigned f = 0; f < count; ++f) {
            Position position;
            bool placed = false;
            for (unsigned attempt = 0; attempt < 10000 && !placed; ++attempt) {
                position = {0.05 + 0.9 * uniform(), 0.05 + 0.9 * uniform()};
                placed = std::none_of(foods.begin(), foods.end(), [&](const Food& food) {
                    return distance(position, food.position) < (food.slime == slime ? 0.16 : 0.045);
                });
            }
            if (!placed) throw std::runtime_error("Could not place separated food sources");
            foods.push_back({position, noNode, slime});
        }
    }
    initializeColonies();
}

void PhysarumSimulation::initializeColonies() {
    iteration = 0;
    balanceError = 0;
    nodes.clear(); parents.clear(); edges.clear(); tips.clear(); retired.clear();
    slimes.clear();
    for (const auto& food : foods) slimes.resize(std::max(slimes.size(), std::size_t(food.slime + 1)));
    tips.reserve(settings.maxTips * slimes.size());
    for (std::size_t f = 0; f < foods.size(); ++f) {
        auto& food = foods[f];
        food.node = addNode(food.position, 0, food.slime);
        slimes[food.slime].foods.push_back(f);
    }
    for (unsigned fan = 0; fan < 3; ++fan)
        for (std::size_t f = 0; f < foods.size(); ++f)
            if (countTips(foods[f].slime) < settings.maxTips) launchFoodTip(f, fan);
    spawnCredits.assign(slimes.size(), 0);
    injections.assign(nodes.size(), 0);
    for (auto& slime : slimes) slime.history.assign(1, slime.foods.size() >= 2
        ? distance(foods[slime.foods[0]].position, foods[slime.foods[1]].position) : 0);
}

void PhysarumSimulation::setTipLimit(unsigned limit) {
    if (limit == 0 || limit > 128) throw std::invalid_argument("Tip limit must be between 1 and 128");
    settings.maxTips = limit;
    tips.reserve(limit * slimes.size());
    std::vector<unsigned> counts(slimes.size());
    tips.erase(std::remove_if(tips.begin(), tips.end(), [&](const Tip& tip) {
        return counts[nodes[tip.node].slime]++ >= limit;
    }), tips.end());
    if (!nodes.empty()) { updateExploration(); transport(); }
}

std::size_t PhysarumSimulation::component(std::size_t node) const {
    while (parents[node] != node) node = parents[node];
    return node;
}

void PhysarumSimulation::rebuildComponents() { parents = components(adjacency(nodes, edges)); }

std::size_t PhysarumSimulation::addNode(Position position, double pressure, unsigned slime) {
    const auto index = nodes.size();
    nodes.push_back({position, pressure, iteration, slime});
    parents.push_back(index);
    return index;
}

unsigned PhysarumSimulation::countTips(unsigned slime) const {
    return std::count_if(tips.begin(), tips.end(), [&](const Tip& tip) {
        return tip.active && nodes[tip.node].slime == slime;
    });
}

void PhysarumSimulation::launchFoodTip(std::size_t food, unsigned fan) {
    Position center{};
    const auto& sources = slimes[foods[food].slime].foods;
    for (auto f : sources) center = add(center, scale(foods[f].position, 1.0 / sources.size()));
    auto direction = iteration == 0
        ? rotate(unit(subtract(center, foods[food].position)), (int(fan) - 1) * 0.8 + (uniform() - 0.5) * 0.2)
        : randomDirection();
    const auto node = foods[food].node;
    tips.push_back({node, chooseTarget(node, direction), food, direction, 0, 0, true});
}

bool PhysarumSimulation::sampleSpawnSite(unsigned slime, SpawnSite& site) {
    struct Candidate { SpawnSite site; double end, weight; };
    std::vector<Candidate> candidates;
    const auto clear = [&](Position position, std::size_t except = noNode) {
        for (std::size_t t = 0; t < tips.size(); ++t)
            if (t != except && tips[t].active && nodes[tips[t].node].slime == slime &&
                distance(position, nodes[tips[t].node].position) < spawnSpacing) return false;
        return true;
    };
    for (auto f : slimes[slime].foods) if (clear(foods[f].position))
        candidates.push_back({{SpawnOrigin::Food, f}, 0, maxStep});

    // Sample the available physical length, rather than tube/node counts.
    // Remove intervals within the same exclusion radius used for food births.
    for (std::size_t e = 0; e < edges.size(); ++e) {
        const auto& edge = edges[e];
        if (!edge.active || edge.state != TubeState::Established || edge.foodLoad < 0.01 || nodes[edge.a].slime != slime) continue;
        const auto a = nodes[edge.a].position;
        const auto ray = subtract(nodes[edge.b].position, a);
        const double length2 = dot(ray, ray);
        std::vector<std::pair<double, double>> intervals{{0, 1}};
        for (const auto& tip : tips) if (tip.active && nodes[tip.node].slime == slime) {
            const auto offset = subtract(a, nodes[tip.node].position);
            const double b = dot(offset, ray);
            const double d = b * b - length2 * (dot(offset, offset) - spawnSpacing * spawnSpacing);
            if (d <= 0) continue;
            const double lo = std::max(0.0, (-b - std::sqrt(d)) / length2);
            const double hi = std::min(1.0, (-b + std::sqrt(d)) / length2);
            if (lo >= hi) continue;
            std::vector<std::pair<double, double>> remaining;
            for (auto interval : intervals) {
                if (hi <= interval.first || lo >= interval.second) remaining.push_back(interval);
                else {
                    if (interval.first < lo) remaining.push_back({interval.first, lo});
                    if (hi < interval.second) remaining.push_back({hi, interval.second});
                }
            }
            intervals.swap(remaining);
            if (intervals.empty()) break;
        }
        for (auto interval : intervals) if (interval.second - interval.first > 1e-12)
            candidates.push_back({{SpawnOrigin::Trunk, e, interval.first}, interval.second,
                                  edge.length * (interval.second - interval.first)});
    }
    for (std::size_t t = 0; t < tips.size(); ++t) {
        const auto& tip = tips[t];
        if (tip.active && nodes[tip.node].slime == slime && tip.sinceBranch >= 6 && clear(nodes[tip.node].position, t))
            candidates.push_back({{SpawnOrigin::Frontier, t}, 0, maxStep});
    }
    double total = 0;
    for (const auto& candidate : candidates) total += candidate.weight;
    if (total == 0) return false;
    double draw = uniform() * total;
    for (const auto& candidate : candidates) {
        if (draw < candidate.weight) {
            site = candidate.site;
            if (site.origin == SpawnOrigin::Trunk)
                site.fraction += (candidate.end - site.fraction) * draw / candidate.weight;
            return true;
        }
        draw -= candidate.weight;
    }
    return false;
}

std::size_t PhysarumSimulation::splitTube(std::size_t index, double fraction) {
    const auto old = edges[index];
    if (fraction * old.length < 1e-7) return old.a;
    if ((1 - fraction) * old.length < 1e-7) return old.b;
    const auto position = add(nodes[old.a].position, scale(subtract(nodes[old.b].position, nodes[old.a].position), fraction));
    const auto node = addNode(position, nodes[old.a].pressure * (1 - fraction) + nodes[old.b].pressure * fraction, nodes[old.a].slime);
    parents[node] = component(old.a);
    edges[index].b = node; edges[index].length = distance(nodes[old.a].position, position);
    auto tail = old; tail.a = node; tail.length = distance(position, nodes[old.b].position);
    edges.push_back(tail);
    return node;
}

void PhysarumSimulation::launchProbe(const SpawnSite& site) {
    if (site.origin == SpawnOrigin::Food) { launchFoodTip(site.index); return; }
    if (site.origin == SpawnOrigin::Frontier) {
        auto& parent = tips[site.index];
        const double angle = (uniform() < 0.5 ? -1 : 1) * (0.55 + 0.35 * uniform());
        const auto direction = rotate(parent.direction, angle);
        const auto target = chooseTarget(parent.node, direction, parent.target);
        parent.sinceBranch = 0;
        tips.push_back({parent.node, target, parent.food, direction, 0, 0, true, SpawnOrigin::Frontier});
        return;
    }
    const auto edge = edges[site.index];
    const auto direction = rotate(unit(subtract(nodes[edge.b].position, nodes[edge.a].position)),
                                  (uniform() < 0.5 ? -1 : 1) * (1.35 + 0.4 * uniform()));
    const auto before = nodes.size();
    const auto node = splitTube(site.index, site.fraction);
    if (nodes.size() > before) nodes[node].sproutJunction = true;
    std::size_t food = noNode;
    double nearest = 2;
    for (std::size_t f = 0; f < foods.size(); ++f) if (component(foods[f].node) == component(node)) {
        const double d = distance(nodes[node].position, foods[f].position);
        if (d < nearest) { food = f; nearest = d; }
    }
    tips.push_back({node, chooseTarget(node, direction), food, direction, 0, 0, true, SpawnOrigin::Trunk});
}

void PhysarumSimulation::spawnTips() {
    for (std::size_t offset = 0; offset < slimes.size() && nodes.size() < settings.maxNodes; ++offset) {
        const unsigned slime = (iteration + offset) % slimes.size();
        auto& credit = spawnCredits[slime];
        credit = std::min(std::max(1.0, settings.spawnRate), credit + settings.spawnRate);
        bool restart = countTips(slime) == 0;
        while (nodes.size() < settings.maxNodes && countTips(slime) < settings.maxTips && (credit >= 1 || restart)) {
            SpawnSite site;
            if (!sampleSpawnSite(slime, site)) break;
            launchProbe(site);
            credit = std::max(0.0, credit - 1);
            restart = false;
        }
    }
}

std::size_t PhysarumSimulation::chooseTarget(std::size_t node, Position direction, std::size_t avoid) const {
    std::size_t selected = noNode;
    double best = -1e30;
    const auto root = component(node);
    const auto slime = nodes[node].slime;
    const bool separate = std::any_of(foods.begin(), foods.end(), [&](const Food& food) {
        return food.slime == slime && component(food.node) != root;
    });
    for (std::size_t i = 0; i < foods.size(); ++i) {
        if (foods[i].slime != slime) continue;
        if ((separate && component(foods[i].node) == root) || distance(nodes[node].position, foods[i].position) < 2 * maxStep) continue;
        const auto delta = subtract(foods[i].position, nodes[node].position);
        const double score = dot(direction, unit(delta)) + 0.1 / (0.1 + distance(nodes[node].position, foods[i].position)) -
                             (i == avoid ? 0.4 : 0);
        if (score > best) { best = score; selected = i; }
    }
    return selected;
}

bool PhysarumSimulation::addEdge(std::size_t a, std::size_t b) {
    if (nodes[a].slime != nodes[b].slime) return false;
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
    const auto slime = nodes[tip.node].slime;
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
        if (foods[i].slime != slime) continue;
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
        double nearest = contactRadius;
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i].slime != slime || i == tip.node || adjacent(i)) continue;
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
        if (!edge.active || nodes[edge.a].slime != slime || edge.a == tip.node || edge.b == tip.node) continue;
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
    auto end = next;
    if (hit != noNode) {
        const auto intersection = add(from, scale(ray, first));
        const auto old = edges[hit];
        if (distance(intersection, nodes[old.a].position) < 1e-6 && distance(from, nodes[old.a].position) <= maxStep)
            destination = old.a;
        else if (distance(intersection, nodes[old.b].position) < 1e-6 && distance(from, nodes[old.b].position) <= maxStep)
            destination = old.b;
        else destination = noNode;
        end = destination == noNode ? intersection : nodes[destination].position;
    }
    // Stop before changing either graph. Only the actual extension up to an
    // own-slime junction is tested; foreign explorers remain transparent.
    for (const auto& trunk : edges) {
        if (!trunk.active || trunk.state != TubeState::Established || nodes[trunk.a].slime == slime) continue;
        const auto a = nodes[trunk.a].position, b = nodes[trunk.b].position;
        if (std::max(from.x, end.x) + contactRadius < std::min(a.x, b.x) ||
            std::min(from.x, end.x) - contactRadius > std::max(a.x, b.x) ||
            std::max(from.y, end.y) + contactRadius < std::min(a.y, b.y) ||
            std::min(from.y, end.y) - contactRadius > std::max(a.y, b.y)) continue;
        if (trunksConflict(from, end, a, b)) {
            tip.active = false;
            return true;
        }
    }
    if (hit != noNode && destination == noNode) {
        const auto& old = edges[hit];
        destination = splitTube(hit, distance(nodes[old.a].position, end) / old.length);
    }
    if (destination == noNode) {
        destination = addNode(next, nodes[tip.node].pressure, slime);
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
    const auto first = count ? iteration % count : 0;
    for (std::size_t offset = 0; offset < count && nodes.size() < settings.maxNodes; ++offset) {
        const auto i = (first + offset) % count;
        auto& tip = tips[i];
        if (!tip.active) continue;
        if (!extend(tip)) { tip.active = false; continue; }
        ++tip.age; ++tip.sinceBranch;
    }
    tips.erase(std::remove_if(tips.begin(), tips.end(), [](const Tip& tip) { return !tip.active; }), tips.end());
    if (nodes.size() < settings.maxNodes) spawnTips();
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
    std::vector<bool> pending(edges.size());
    for (std::size_t e = 0; e < edges.size(); ++e)
        pending[e] = edges[e].active && !peeled[e] && edges[e].state != TubeState::Established;
    if (commitPromotions(pending, anchors)) {
        // Rejected routes have been severed. Reclaim disconnected exploration
        // and rebuild protection/food components before the hydraulic solve.
        retire();
        updateExploration();
        return;
    }
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

bool PhysarumSimulation::commitPromotions(const std::vector<bool>& pending, const std::vector<bool>& anchors) {
    if (std::none_of(pending.begin(), pending.end(), [](bool candidate) { return candidate; })) return false;
    if (slimes.size() == 1) {
        for (std::size_t e = 0; e < edges.size(); ++e) if (pending[e]) edges[e].state = TubeState::Established;
        return false;
    }
    constexpr int resolution = 64;
    const auto cell = [](double coordinate) { return std::clamp(int(std::floor(coordinate * resolution)), 0, resolution - 1); };
    std::vector<std::vector<std::size_t>> bins(resolution * resolution), incident(nodes.size());
    const auto insert = [&](std::size_t e) {
        const auto a = nodes[edges[e].a].position, b = nodes[edges[e].b].position;
        for (int y = cell(std::min(a.y, b.y)); y <= cell(std::max(a.y, b.y)); ++y)
            for (int x = cell(std::min(a.x, b.x)); x <= cell(std::max(a.x, b.x)); ++x)
                bins[y * resolution + x].push_back(e);
    };
    for (std::size_t e = 0; e < edges.size(); ++e) {
        if (edges[e].active && edges[e].state == TubeState::Established) insert(e);
        if (pending[e]) { incident[edges[e].a].push_back(e); incident[edges[e].b].push_back(e); }
    }
    std::vector<bool> processed(edges.size());
    std::vector<unsigned> checked(edges.size());
    unsigned stamp = 0;
    bool rejected = false;
    // Existing trunks have priority. Simultaneous claims use a rotating slime
    // order; an accepted route becomes an obstacle for subsequent claims.
    for (std::size_t offset = 0; offset < slimes.size(); ++offset) {
        const auto slime = (iteration + offset) % slimes.size();
        for (std::size_t start = 0; start < edges.size(); ++start) {
            if (!pending[start] || processed[start] || nodes[edges[start].a].slime != slime) continue;
            std::vector<std::size_t> route;
            std::queue<std::size_t> queue;
            queue.push(start); processed[start] = true;
            while (!queue.empty()) {
                const auto e = queue.front(); queue.pop(); route.push_back(e);
                for (auto node : {edges[e].a, edges[e].b}) if (!anchors[node])
                    for (auto neighbor : incident[node]) if (!processed[neighbor]) {
                        processed[neighbor] = true; queue.push(neighbor);
                    }
            }
            bool conflict = false;
            for (auto e : route) {
                ++stamp;
                const auto a = nodes[edges[e].a].position, b = nodes[edges[e].b].position;
                for (int y = cell(std::min(a.y, b.y) - contactRadius); y <= cell(std::max(a.y, b.y) + contactRadius) && !conflict; ++y)
                    for (int x = cell(std::min(a.x, b.x) - contactRadius); x <= cell(std::max(a.x, b.x) + contactRadius) && !conflict; ++x)
                        for (auto other : bins[y * resolution + x]) {
                            if (checked[other] == stamp) continue;
                            checked[other] = stamp;
                            const auto& trunk = edges[other];
                            if (nodes[trunk.a].slime == slime) continue;
                            if (trunksConflict(a, b, nodes[trunk.a].position, nodes[trunk.b].position)) { conflict = true; break; }
                        }
                if (conflict) break;
            }
            if (conflict) {
                rejected = true;
                for (auto e : route) {
                    edges[e].active = false; edges[e].state = TubeState::Abandoned;
                    edges[e].flow = edges[e].foodLoad = 0;
                }
            } else for (auto e : route) {
                edges[e].state = TubeState::Established;
                insert(e);
            }
        }
    }
    return rejected;
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

void PhysarumSimulation::coalesceSproutJunctions() {
    std::vector<std::vector<std::size_t>> incident(nodes.size());
    std::vector<bool> pinned(nodes.size());
    for (const auto& food : foods) pinned[food.node] = true;
    for (const auto& tip : tips) if (tip.active) pinned[tip.node] = true;
    for (std::size_t e = 0; e < edges.size(); ++e) if (edges[e].active) {
        incident[edges[e].a].push_back(e); incident[edges[e].b].push_back(e);
    }
    std::queue<std::size_t> queue;
    for (std::size_t n = 0; n < nodes.size(); ++n) if (nodes[n].sproutJunction && !pinned[n]) queue.push(n);
    while (!queue.empty()) {
        const auto node = queue.front(); queue.pop();
        std::vector<std::size_t> live;
        for (auto e : incident[node]) if (edges[e].active && (edges[e].a == node || edges[e].b == node) &&
            std::find(live.begin(), live.end(), e) == live.end()) live.push_back(e);
        if (live.size() != 2) continue;
        const auto first = edges[live[0]], second = edges[live[1]];
        if (first.state != TubeState::Established || second.state != TubeState::Established) continue;
        const auto a = first.a == node ? first.b : first.a;
        const auto b = second.a == node ? second.b : second.a;
        const double length = distance(nodes[a].position, nodes[b].position);
        if (a == b || length > maxStep + 1e-12 || std::abs(length - first.length - second.length) > 1e-12) continue;
        if (std::any_of(incident[a].begin(), incident[a].end(), [&](std::size_t e) {
            return edges[e].active && ((edges[e].a == a && edges[e].b == b) || (edges[e].a == b && edges[e].b == a));
        })) continue;
        auto& merged = edges[live[0]];
        merged.a = a; merged.b = b; merged.length = length;
        merged.conductivity = length / (first.length / first.conductivity + second.length / second.conductivity);
        merged.flow = first.b == node ? first.flow : -first.flow;
        merged.born = std::min(first.born, second.born);
        merged.weakSteps = std::min(first.weakSteps, second.weakSteps);
        edges[live[1]].active = false;
        incident[a].push_back(live[0]); incident[b].push_back(live[0]);
        if (nodes[a].sproutJunction && !pinned[a]) queue.push(a);
        if (nodes[b].sproutJunction && !pinned[b]) queue.push(b);
    }
}

void PhysarumSimulation::retire() {
    coalesceSproutJunctions();
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
            retired.push_back({nodes[edge.a].position, nodes[edge.b].position, nodes[edge.a].slime});
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
            points[edge.a].slime != points[edge.b].slime ||
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
    std::vector<bool> driven(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(supply[i])) throw std::invalid_argument("Flow injections must be finite");
        sum[roots[i]] += supply[i];
        driven[roots[i]] = driven[roots[i]] || supply[i] != 0;
    }
    for (auto total : sum) if (std::abs(total) > 1e-8)
        throw std::invalid_argument("Flow supply must balance demand within each component");
    // A food-pair basis solve drives one component. Other components have the
    // exact zero-pressure solution and need no conjugate-gradient iterations.
    for (std::size_t i = 0; i < n; ++i) {
        if (driven[roots[i]] && roots[i] != i) {
            unknowns.push_back(i);
            diagonal[i] = 0;
            for (const auto& neighbor : neighbors[i]) diagonal[i] += neighbor.second;
            x[i] = points[i].pressure - points[roots[i]].pressure;
        } else rhs[i] = 0;
    }
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
    std::vector<std::vector<std::size_t>> groups(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) groups[component(i)].push_back(i);
    for (auto& slime : slimes) {
        double closest = slime.history.back();
        if (slime.foods.size() >= 2) {
            const auto a = component(foods[slime.foods[0]].node), b = component(foods[slime.foods[1]].node);
            if (a == b) closest = 0;
            else if (closest > 0)
                for (auto i : groups[a]) for (auto j : groups[b])
                    closest = std::min(closest, distance(nodes[i].position, nodes[j].position));
        }
        slime.history.push_back(closest);
        if (slime.history.size() > progressHistoryLimit) slime.history.pop_front();
    }
}
unsigned PhysarumSimulation::getFoodGroups(std::size_t slime) const {
    std::vector<std::size_t> roots;
    for (const auto& food : foods) if (slime == noNode || food.slime == slime) roots.push_back(component(food.node));
    std::sort(roots.begin(), roots.end());
    return std::unique(roots.begin(), roots.end()) - roots.begin();
}
unsigned PhysarumSimulation::getJoinedSlimes() const {
    unsigned joined = 0;
    for (std::size_t slime = 0; slime < slimes.size(); ++slime) joined += getFoodGroups(slime) == 1;
    return joined;
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
double PhysarumSimulation::getConnectionLength(bool transportOnly, unsigned slime) const {
    const auto& sources = slimes.at(slime).foods;
    if (sources.size() < 2) return std::numeric_limits<double>::infinity();
    const auto start = foods[sources[0]].node, goal = foods[sources[1]].node;
    if (component(start) != component(goal))
        return std::numeric_limits<double>::infinity();
    std::vector<double> lengths(nodes.size(), std::numeric_limits<double>::infinity());
    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    lengths[start] = 0; queue.push({0, start});
    std::vector<std::vector<std::pair<std::size_t, double>>> neighbors(nodes.size());
    for (const auto& edge : edges) if (edge.active &&
        (!transportOnly || (edge.foodLoad >= 0.01 && edge.conductivity >= 0.002))) {
        neighbors[edge.a].push_back({edge.b, edge.length}); neighbors[edge.b].push_back({edge.a, edge.length});
    }
    while (!queue.empty()) {
        const auto [length, node] = queue.top(); queue.pop();
        if (node == goal) return length;
        if (length > lengths[node]) continue;
        for (const auto& neighbor : neighbors[node]) if (length + neighbor.second < lengths[neighbor.first]) {
            lengths[neighbor.first] = length + neighbor.second;
            queue.push({lengths[neighbor.first], neighbor.first});
        }
    }
    return std::numeric_limits<double>::infinity();
}
