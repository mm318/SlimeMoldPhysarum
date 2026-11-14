#include "PhysarumSimulation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

// Deterministic geometries exercise production contact handling without relying
// on a random seed to arrange an exact endpoint or interior intersection.
struct PhysarumSimulationTestAccess {
    using S = PhysarumSimulation;
    static void load(S& s, std::vector<S::Node> nodes,
                     const std::vector<std::pair<std::size_t, std::size_t>>& links,
                     std::vector<S::Tip> tips) {
        S::Settings settings; settings.sixFoods = false;
        s.reset(settings, 2026);
        s.nodes = std::move(nodes); s.edges.clear(); s.rebuildComponents();
        s.foods = {{s.nodes[0].position, 0}, {s.nodes[1].position, 1}};
        s.tips = std::move(tips); s.tips.reserve(settings.maxTips);
        for (auto link : links) if (!s.addEdge(link.first, link.second))
            throw std::runtime_error("Invalid contact fixture");
        s.iteration = 100;
        s.updateExploration(); s.transport();
    }
    static bool extend(S& s, std::size_t tip) {
        const bool result = s.extend(s.tips.at(tip));
        s.updateExploration(); s.transport();
        return result;
    }
    static void finishFrontiers(S& s) { s.tips.clear(); s.updateExploration(); }
    static void settle(S& s, unsigned steps) {
        for (unsigned t = 0; t < steps; ++t) {
            s.adaptTubes(s.edges, s.settings, s.iteration);
            s.retire(); s.updateExploration(); s.transport(); ++s.iteration;
        }
    }
};

namespace {
using Simulation = PhysarumSimulation;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void checkHydraulics() {
    std::vector<Simulation::Node> nodes{{{0, 0}}, {{0.005, 0}}, {{0.01, 0}}, {{0.005, 0.005}}};
    const double diagonal = std::sqrt(0.005 * 0.005 * 2);
    std::vector<Simulation::Edge> edges{{0, 1, 0.005, 1}, {1, 2, 0.005, 1},
                                      {0, 3, diagonal, 1}, {3, 2, diagonal, 1}};
    const std::vector<double> supply{1, 0, -1, 0};
    require(Simulation::solveFlow(nodes, edges, supply) < 1e-10, "Kirchhoff flow must conserve mass");
    const double shortFlow = 1 / (1 + 0.01 / (2 * diagonal));
    require(std::abs(edges[0].flow - shortFlow) < 1e-9 &&
            std::abs(edges[2].flow - (1 - shortFlow)) < 1e-9,
            "Parallel tubes must split flow inversely to their resistance");
    const auto forward = edges;
    require(Simulation::solveFlow(nodes, edges, {-1, 0, 1, 0}) < 1e-10, "Reverse flow must also conserve mass");
    for (std::size_t i = 0; i < edges.size(); ++i)
        require(std::abs(edges[i].flow + forward[i].flow) < 1e-9,
                "Switching food supply roles must reverse transport without changing tube geometry");
    // Apply the same conductance adaptation used by the simulation to a known
    // competing-route fixture. The shorter route must win actual flow, not color.
    for (int t = 0; t < 600; ++t) {
        Simulation::solveFlow(nodes, edges, supply);
        for (auto& edge : edges) edge.foodLoad = std::abs(edge.flow);
        Simulation::adaptTubes(edges, {}, t + 1);
    }
    require(Simulation::solveFlow(nodes, edges, supply) < 1e-8 && edges[0].flow > 0.95 && edges[2].flow < 0.05,
            "Reinforcement must transfer transport to the shorter competing route");
    require(edges[0].conductivity > 10 * edges[2].conductivity, "Useful tubes must become thicker than unused tubes");
    bool rejected = false;
    try { Simulation::solveFlow(nodes, edges, {1, 0, 0, 0}); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Unbalanced hydraulic supply must be rejected");
    nodes = {{{0, 0}, 100}, {{0.01, 0}, 101}, {{0.8, 0}, -30}, {{0.81, 0}, -32}, {{0.5, 0.8}, 200}};
    edges = {{0, 1, 0.01, 1}, {2, 3, 0.01, 1}};
    require(Simulation::solveFlow(nodes, edges, {0.01, -0.01, 0.02, -0.02, 0}) < 1e-10 &&
            std::abs(edges[0].flow - 0.01) < 1e-10 && std::abs(edges[1].flow - 0.02) < 1e-10,
            "Separate colonies must solve independent balanced flows despite different pressure offsets");
    rejected = false;
    try { Simulation::solveFlow(nodes, edges, {0.01, 0, -0.01, 0, 0}); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "A globally balanced supply must not transport flow across an unjoined gap");
}

void checkRouteFairness() {
    const double diagonal = std::hypot(0.005, 0.005);
    const std::vector<Simulation::Node> geometry{{{0, 0}}, {{0.005, 0}}, {{0.01, 0}}, {{0.005, 0.005}}};
    for (bool sixFoods : {false, true}) for (bool shortEstablished : {true, false}) {
        auto nodes = geometry;
        const double shortD = shortEstablished ? 1 : 0.003, longD = shortEstablished ? 0.003 : 1;
        std::vector<Simulation::Edge> edges{{0, 1, 0.005, shortD}, {1, 2, 0.005, shortD},
                                          {0, 3, diagonal, longD}, {3, 2, diagonal, longD}};
        Simulation::Settings settings;
        std::vector<std::size_t> ports{0, 2};
        if (sixFoods) {
            for (Simulation::Position p : {Simulation::Position{-0.005, 0}, {0, -0.005}, {0.015, 0}, {0.01, -0.005}}) {
                const auto node = nodes.size();
                ports.push_back(node); nodes.push_back({p});
                edges.push_back({node < 6 ? 0u : 2u, node, 0.005, 1});
            }
        }
        for (std::size_t e = 0; e < edges.size(); ++e)
            edges[e].born = e < 4 && ((e < 2) != shortEstablished) ? 100 : 0;
        for (unsigned t = 0; t < 1800; ++t) {
            for (auto& edge : edges) edge.foodLoad = 0;
            const double pairs = ports.size() * (ports.size() - 1) / 2;
            for (std::size_t a = 0; a < ports.size(); ++a) for (std::size_t b = a + 1; b < ports.size(); ++b) {
                std::vector<double> supply(nodes.size()); supply[ports[a]] = 1; supply[ports[b]] = -1;
                require(Simulation::solveFlow(nodes, edges, supply) < 1e-8, "Competition fixture must conserve flow");
                if (shortEstablished && a == 0 && b == 1)
                    require(edges[0].flow > 0.95, "A new longer route must never dethrone an established shorter route");
                for (auto& edge : edges) edge.foodLoad += std::abs(edge.flow) / pairs;
            }
            Simulation::adaptTubes(edges, settings, 100 + t);
            if (t == 30) require(std::all_of(edges.begin(), edges.end(), [](const auto& e) { return e.active; }),
                                 "A new mesh connection must allow both routes to compete before pruning");
        }
        require(edges[0].active && edges[1].active && !edges[2].active && !edges[3].active,
                "Hydraulic adaptation must retire the inefficient side of the mesh regardless of which route is older");
        std::vector<double> supply(nodes.size()); supply[0] = 1; supply[2] = -1;
        Simulation::solveFlow(nodes, edges, supply);
        require(edges[0].conductivity > 100 * edges[2].conductivity && edges[0].flow > 0.99,
                "Mesh selection must transfer real conductivity and flow to the shorter route");
        for (std::size_t i = 0; i < geometry.size(); ++i)
            require(nodes[i].position == geometry[i].position, "Mesh selection must not move established tube nodes");
    }
}

void checkBackboneSettlesWhileExploring() {
    Simulation simulation;
    Simulation::Settings settings;
    settings.sixFoods = false;
    settings.maxTips = 48;
    settings.branching = 0.12;
    simulation.reset(settings, 2026);
    double low = 10, high = 0, initial = 10, rebound = 0;
    for (unsigned turn = 0; turn < 2500; ++turn) {
        simulation.step();
        const double route = simulation.getConnectionLength(true);
        if (initial == 10 && std::isfinite(route)) initial = route;
        if (turn > 400) {
            require(std::isfinite(route) && route <= initial + 1e-9,
                    "Continued branching must not replace the initial food-carrying backbone with a longer route");
            if (turn > 2000) {
                low = std::min(low, route); high = std::max(high, route);
                rebound = std::max(rebound, route - low);
            }
        }
        require(simulation.getBalanceError() < 1e-5, "Long-running adaptation must retain accurate pressure/flow balance");
    }
    std::cout << "Two-food backbone during continued exploration: initial=" << initial
              << ", late=" << low << ".." << high << ", rebound=" << rebound << '\n';
    // Discovering a shorter physical connection can improve the mature route
    // during this window; reject inefficient replacements, not improvements.
    require(rebound < 0.002 && high <= initial && !simulation.getTips().empty(),
            "Continued exploration must preserve the mature backbone while permitting newly discovered improvements");
}

void checkLongGrowthAndFusion() {
    Simulation simulation;
    Simulation::Settings settings;
    settings.sixFoods = false; settings.maxTips = 1;
    settings.maxNodes = 500; settings.branching = 0;
    simulation.reset(settings, 2026);
    require(simulation.getFoodGroups() == 2 && simulation.getHistory().back() > 0.8,
            "Seeding food B must not count as a physical A-B connection");
    simulation.step();
    require(simulation.getEdges().size() == 1 &&
            std::abs(simulation.getEdges()[0].conductivity - 0.003) < 1e-12 &&
            simulation.getEdges()[0].flow == 0 && simulation.getEdges()[0].state == Simulation::TubeState::Exploring,
            "A newborn exploring branch must remain thin and alive without consuming flow");
    unsigned joined = 0;
    bool oldBranch = false, otherFoodLaunched = false;
    for (unsigned turn = 1; turn < 180; ++turn) {
        simulation.step();
        if (simulation.getFoodGroups() == 1 && !joined) joined = simulation.getIteration();
        if (!joined && turn > 40)
            for (const auto& edge : simulation.getEdges()) {
                oldBranch |= turn - edge.born > 31;
                require(edge.state == Simulation::TubeState::Exploring && edge.conductivity == 0.003 && edge.flow == 0,
                        "An unconnected frontier's entire branch must survive with zero flow regardless of age");
            }
        for (const auto& tip : simulation.getTips()) otherFoodLaunched |= tip.food == 1;
    }
    require(oldBranch && joined >= 84 && joined < 120 && otherFoodLaunched,
            "Flow-free exploration must outlive 31 steps, physically connect, and rotate the single tip slot");
}

void checkContactSemantics() {
    using Access = PhysarumSimulationTestAccess;
    Simulation tips;
    Access::load(tips, {{{0.48, 0.5}}, {{0.524, 0.5}}, {{0.49, 0.5}}, {{0.499, 0.5}},
                        {{0.514, 0.5}}, {{0.505, 0.5}}},
                 {{0, 2}, {2, 3}, {1, 4}, {4, 5}},
                 {{3, 1, 0, {1, 0}, 1}, {5, 0, 1, {-1, 0}, 1}});
    require(Access::extend(tips, 0), "Head-on exploring tips must join even when their segments are collinear");
    require(!tips.getTips()[0].active && !tips.getTips()[1].active,
            "Tip-to-tip contact must consume both frontiers immediately");
    require(tips.getFoodGroups() == 1 && tips.getEdges().size() == 5 &&
            std::all_of(tips.getEdges().begin(), tips.getEdges().end(), [](const auto& e) { return e.state == Simulation::TubeState::Established; }),
            "Both complete branches must enter hydraulic competition at tip-to-tip fusion");
    for (const auto& edge : tips.getEdges()) require(std::abs(std::abs(edge.flow) - 1) < 1e-8,
                                                    "The new trunk must carry actual food-to-food flow");

    Simulation branch;
    Access::load(branch, {{{0.485, 0.495}}, {{0.512, 0.47}}, {{0.495, 0.495}}, {{0.505, 0.495}},
                          {{0.512, 0.48}}, {{0.512, 0.49}}, {{0.512, 0.50}}, {{0.512, 0.51}}},
                 {{0, 2}, {2, 3}, {1, 4}, {4, 5}, {5, 6}, {6, 7}},
                 {{3, 1, 0, {1, 0}, 1}, {7, 0, 1, {0, 1}, 1}});
    require(Access::extend(branch, 0), "An exploring tip must make a junction inside another exploring branch");
    require(!branch.getTips()[0].active && branch.getTips()[1].active && branch.getTips()[1].node == 7,
            "An interior branch contact must consume only the incoming tip");
    unsigned exploring = 0;
    double protectedLength = 0;
    for (const auto& edge : branch.getEdges()) if (edge.state == Simulation::TubeState::Exploring) {
        ++exploring; protectedLength += edge.length;
        require(edge.flow == 0 && edge.foodLoad == 0, "The surviving distal branch must draw no hydraulic flow");
    }
    require(exploring == 2 && std::abs(protectedLength - 0.015) < 1e-10 && branch.getFoodGroups() == 1,
            "Only the distal subset beyond the new junction must retain exploration protection");
    auto copy = branch.getEdges();
    for (unsigned t = 101; t < 250; ++t) {
        Simulation::adaptTubes(copy, branch.getSettings(), t);
        for (const auto& edge : copy) if (edge.state == Simulation::TubeState::Exploring)
            require(edge.active && edge.conductivity == 0.003, "Zero-flow distal segments must remain alive");
    }
    Access::finishFrontiers(branch);
    Access::settle(branch, 100);
    require(!branch.getRetiredEdges().empty(), "An abandoned zero-flow branch must become eligible for retirement");
}

void checkSharedForkProtection() {
    Simulation fork;
    PhysarumSimulationTestAccess::load(fork,
        {{{0.48, 0.5}}, {{0.50, 0.5}}, {{0.49, 0.5}}, {{0.49, 0.51}}, {{0.49, 0.52}}, {{0.50, 0.51}}},
        {{0, 2}, {2, 1}, {2, 3}, {3, 4}, {3, 5}},
        {{4, 1, 0, {0, 1}, 1}, {5, 1, 0, {1, 0}, 1}});
    fork.setTipLimit(1);
    const auto& edges = fork.getEdges();
    require(edges[2].state == Simulation::TubeState::Exploring && edges[3].state == Simulation::TubeState::Exploring &&
            edges[4].state == Simulation::TubeState::Abandoned,
            "Removing a fork's side tip must release its stub while retaining the surviving tip's shared stem");
    for (const auto& edge : edges) {
        const auto& nodes = fork.getNodes();
        const double expected = edge.conductivity / edge.length * (nodes[edge.a].pressure - nodes[edge.b].pressure);
        require(std::abs(edge.flow - expected) < 1e-9,
                "Protected and abandoned fork portions must share their trunk anchor's pressure");
    }
    PhysarumSimulationTestAccess::settle(fork, 100);
    require(fork.getNodes().size() == 5 && fork.getEdges().size() == 4 && fork.getTips().size() == 1,
            "An abandoned fork must release its node without destroying the surviving protected branch");
}

void checkGrowth(bool sixFoods, unsigned seed) {
    Simulation simulation;
    Simulation::Settings settings;
    settings.sixFoods = sixFoods;
    simulation.reset(settings, seed);
    require(simulation.getNodes().size() == (sixFoods ? 6u : 2u) && simulation.getEdges().empty(),
            "Growth must begin with one isolated root per food and no precomputed edges");
    require(simulation.getFoodGroups() == simulation.getFoods().size(),
            "Every food must initially be an independent colony");
    std::vector<unsigned> initialTips(simulation.getFoods().size());
    for (const auto& tip : simulation.getTips()) {
        require(tip.node == simulation.getFoods()[tip.food].node, "Initial tips must start at their supplying food");
        ++initialTips[tip.food];
    }
    require(std::all_of(initialTips.begin(), initialTips.end(), [](unsigned count) { return count == 3; }),
            "The default budget must give every food three initial tips");
    unsigned firstGoal = 0;
    bool loop = false, branching = false, continuedExploration = false, lateral = false;
    for (unsigned turn = 0; turn < 1200; ++turn) {
        std::set<std::tuple<unsigned, double, double>> previousPositions;
        for (const auto& node : simulation.getNodes())
            previousPositions.emplace(node.born, node.position.x, node.position.y);
        require(simulation.step(), "Growth must continue until explicitly paused by its caller");
        const auto& nodes = simulation.getNodes();
        const auto& edges = simulation.getEdges();
        for (const auto& node : nodes) if (node.born < turn)
            require(previousPositions.count({node.born, node.position.x, node.position.y}),
                    "Established nodes must remain fixed through growth, junction splitting, and mesh pruning");
        require(nodes.size() <= settings.maxNodes && simulation.getTips().size() <= settings.maxTips,
                "Growth must respect its node and frontier budgets");
        if (simulation.getBalanceError() >= 1e-5)
            std::cerr << "Balance failure at " << simulation.getIteration() << " with " << nodes.size()
                      << " nodes: " << simulation.getBalanceError() << '\n';
        require(simulation.getBalanceError() < 1e-5, "Node pressures must produce mass-conserving transport");
        std::vector<unsigned> degree(nodes.size());
        std::vector<double> balance(nodes.size());
        for (const auto& edge : edges) {
            require(edge.a < nodes.size() && edge.b < nodes.size() && edge.a != edge.b,
                    "Tubes need distinct existing endpoints");
            const double length = Simulation::distance(nodes[edge.a].position, nodes[edge.b].position);
            require(std::abs(length - edge.length) < 1e-10 && length > 0 && length <= 0.01 + 1e-12,
                    "Every grown, joined, or split tube must obey the Euclidean 0.01 limit");
            if (edge.state == Simulation::TubeState::Exploring)
                require(edge.flow == 0 && edge.foodLoad == 0 && edge.conductivity == 0.003,
                        "Protected exploring branches must remain alive and thin without hydraulic demand");
            require(std::isfinite(edge.conductivity) && edge.conductivity > 0 && std::isfinite(edge.flow),
                    "Tube conductance and flow must remain finite");
            if (edge.active) {
                ++degree[edge.a]; ++degree[edge.b];
                balance[edge.a] += edge.flow; balance[edge.b] -= edge.flow;
                const double expected = edge.conductivity / edge.length *
                    (nodes[edge.a].pressure - nodes[edge.b].pressure);
                require(std::abs(edge.flow - expected) < 1e-9, "Reported flow must use the actual tube conductance");
            } else require(edge.flow == 0, "Pruned tubes must not carry transport");
        }
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto p = nodes[i].position;
            require(std::isfinite(p.x) && std::isfinite(p.y) && p.x >= 0 && p.x <= 1 && p.y >= 0 && p.y <= 1,
                    "The graph must remain on the unit square");
            double nearestFood = 2;
            for (const auto& food : simulation.getFoods()) nearestFood = std::min(nearestFood, Simulation::distance(food.position, p));
            require(nearestFood <= simulation.getIteration() * 0.01 + 1e-10,
                    "A frontier must not jump ahead of physically possible growth from every food");
            require(std::abs(balance[i] - simulation.getInjections()[i]) < 1e-5,
                    "Every junction must balance the actual edge flows and nutrient demand");
            branching |= degree[i] >= 3;
        }
        loop |= edges.size() >= nodes.size();
        if (std::isfinite(simulation.getConnectionLength()) && !firstGoal) firstGoal = simulation.getIteration();
        for (const auto& food : simulation.getFoods())
            require(food.node < nodes.size() && nodes[food.node].position == food.position,
                    "Every food root must remain fixed and alive, even in a separate colony");
        std::vector<bool> isFood(nodes.size());
        for (const auto& food : simulation.getFoods()) isFood[food.node] = true;
        for (std::size_t i = 0; i < nodes.size(); ++i) if (!isFood[i])
            require(simulation.getInjections()[i] == 0, "Exploring tips and junctions must never consume flow");
        continuedExploration |= simulation.getFoodGroups() == 1 && turn > 200 &&
            std::any_of(edges.begin(), edges.end(), [](const auto& edge) { return edge.state == Simulation::TubeState::Exploring; });
        for (const auto& tip : simulation.getTips())
            lateral |= tip.age == 0 && nodes[tip.node].born + 40 < turn &&
                       std::none_of(simulation.getFoods().begin(), simulation.getFoods().end(),
                                    [&](const auto& food) { return food.node == tip.node; });
    }
    if (sixFoods) {
        auto referenceNodes = simulation.getNodes();
        auto referenceEdges = simulation.getEdges();
        const auto& foods = simulation.getFoods();
        std::vector<double> load(referenceEdges.size());
        const double pairs = foods.size() * (foods.size() - 1) / 2;
        for (std::size_t a = 0; a < foods.size(); ++a) for (std::size_t b = a + 1; b < foods.size(); ++b) {
            require(foods[a].node != Simulation::noNode && foods[b].node != Simulation::noNode,
                    "Every food must remain connected for pairwise transport validation");
            std::vector<double> supply(referenceNodes.size());
            supply[foods[a].node] = 1; supply[foods[b].node] = -1;
            require(Simulation::solveFlow(referenceNodes, referenceEdges, supply) < 1e-5,
                    "Independent pairwise pressure solves must conserve mass");
            for (std::size_t e = 0; e < referenceEdges.size(); ++e) {
                load[e] += std::abs(referenceEdges[e].flow) / pairs;
            }
        }
        for (std::size_t e = 0; e < referenceEdges.size(); ++e) {
            require(std::abs(load[e] - simulation.getEdges()[e].foodLoad) < 1e-6,
                    "Tube reinforcement must average every food pair, independently of the displayed pumping phase");
        }
    }
    if (!sixFoods) require(firstGoal >= 42 && firstGoal < 84,
                          "Opposing A/B fronts must physically meet between the foods without jumping");
    require(simulation.getFoodGroups() == 1, "The seeded colonies must merge into one supplied network");
    require(continuedExploration && lateral,
            "Protected exploration must continue after food connection and sprout from established trunks");
    require(branching && loop, "The growing graph must form branch junctions and reconnecting routes");
    require(!simulation.getRetiredEdges().empty(), "Weak exploratory branches must retire while growth continues");
    require(std::isfinite(simulation.getConnectionLength()) && simulation.getConnectionLength() >= 0.84 - 1e-9,
            "The active graph must retain a real A-B connection");
    if (!sixFoods) require(simulation.getConnectionLength() < 0.9, "The two-food fixture must retain a short transport route");
    // The fully joined working graph must contain only food-supplied live nodes.
    // Retired geometry is stored separately and cannot consume its node budget.
    std::vector<std::vector<std::size_t>> neighbors(simulation.getNodes().size());
    for (const auto& edge : simulation.getEdges()) {
        neighbors[edge.a].push_back(edge.b); neighbors[edge.b].push_back(edge.a);
    }
    std::vector<bool> visited(neighbors.size());
    std::queue<std::size_t> queue; queue.push(0); visited[0] = true;
    while (!queue.empty()) {
        const auto node = queue.front(); queue.pop();
        for (auto next : neighbors[node]) if (!visited[next]) { visited[next] = true; queue.push(next); }
    }
    require(std::all_of(visited.begin(), visited.end(), [](bool v) { return v; }), "Every live branch must remain connected to a food");
    require(simulation.getHistory().size() == 1201 &&
            std::is_sorted(simulation.getHistory().rbegin(), simulation.getHistory().rend()),
            "Best approach history must include the seed and record monotonic progress");
    std::cout << (sixFoods ? "Six" : "Two") << " foods, seed " << seed << ": groups=" << simulation.getFoodGroups()
              << ", nodes=" << simulation.getNodes().size() << ", active=" << simulation.getActiveEdges()
              << ", first B=" << firstGoal << ", A-B=" << simulation.getConnectionLength() << '\n';
    simulation.reset(settings, seed);
    require(simulation.getNodes().size() == simulation.getFoods().size() && simulation.getEdges().empty() && simulation.getRetiredEdges().empty() && simulation.getIteration() == 0,
            "Reset must clear all grown tubes and transport state");
}

void checkRecyclingAndContinuousGrowth() {
    Simulation simulation;
    Simulation::Settings settings;
    settings.maxNodes = 25;
    settings.maxTips = 1;
    settings.sixFoods = false;
    settings.branching = 0;
    settings.reinforcement = 0;
    settings.adaptation = 0.2;
    simulation.reset(settings, 2026);
    unsigned births = 0;
    bool freed = false, lateGrowth = false;
    std::size_t previousNodes = 1;
    for (unsigned turn = 0; turn < 5000; ++turn) {
        // Explicit frontier retirement makes this tiny fixture reclaim capacity;
        // an entirely protected full graph otherwise waits for room by design.
        if (turn % 100 == 99) PhysarumSimulationTestAccess::finishFrontiers(simulation);
        require(simulation.step(), "There must be no automatic growth or total-step cutoff");
        const auto& nodes = simulation.getNodes();
        require(nodes.size() <= settings.maxNodes, "Only current live nodes may consume the node cap");
        freed |= nodes.size() < previousNodes;
        previousNodes = nodes.size();
        for (std::size_t i = 1; i < nodes.size(); ++i) if (nodes[i].born == turn) {
            ++births;
            lateGrowth |= turn > 1200;
        }
        require(simulation.getBalanceError() < 1e-5, "Reclaiming and remapping nodes must preserve hydraulic balance");
        require(simulation.getRetiredEdges().size() <= Simulation::retiredHistoryLimit,
                "Drawing history must remain bounded during an unlimited run");
    }
    require(freed && births > 10 * settings.maxNodes && lateGrowth,
            "Dead nodes must release real capacity for repeated growth beyond the old time and node limits");
    require(simulation.getIteration() == 5000 && simulation.getHistory().size() == Simulation::progressHistoryLimit,
            "Progress history may roll over without stopping simulation growth");
    for (const auto& edge : simulation.getRetiredEdges())
        require(Simulation::distance(edge.a, edge.b) <= Simulation::maxStep + 1e-12,
                "Archived tubes must preserve their real bounded geometry independently of remapped live nodes");
    std::cout << "Controlled recycling: " << births << " births with a " << settings.maxNodes << "-node live cap\n";
}

void checkProtectedNodeBudget() {
    Simulation simulation;
    Simulation::Settings settings;
    settings.sixFoods = false; settings.maxNodes = 25;
    settings.maxTips = 1; settings.branching = 0;
    simulation.reset(settings, 2026);
    for (unsigned t = 0; t < 100; ++t) simulation.step();
    require(simulation.getNodes().size() == 25 && simulation.getTips().size() == 1,
            "A full protected budget must pause growth without destroying its waiting frontier");
    for (const auto& edge : simulation.getEdges())
        require(edge.state == Simulation::TubeState::Exploring && edge.active && edge.flow == 0 && edge.conductivity == 0.003,
                "The node cap must not remove protection or create artificial demand");
}

void checkLiveTipLimit() {
    Simulation simulation;
    Simulation::Settings settings;
    settings.maxTips = 3;
    settings.branching = 1;
    simulation.reset(settings, 42);
    for (int t = 0; t < 15; ++t) simulation.step();
    const auto before = simulation.getNodes();
    const auto turn = simulation.getIteration();
    simulation.setTipLimit(12);
    require(simulation.getIteration() == turn && simulation.getNodes().size() == before.size(),
            "Changing the tip limit must not reset the network or advance time");
    for (std::size_t i = 0; i < before.size(); ++i)
        require(simulation.getNodes()[i].position == before[i].position, "Changing tip capacity must preserve graph geometry");
    bool expanded = false;
    for (int t = 0; t < 30; ++t) {
        simulation.step();
        expanded |= simulation.getTips().size() > 3;
        require(simulation.getTips().size() <= 12, "Increasing the live tip cap must still enforce its new bound");
    }
    require(expanded, "Raising the tip limit must permit additional forks");
    simulation.setTipLimit(1);
    require(simulation.getTips().size() <= 1 && simulation.getSettings().maxTips == 1,
            "Lowering the tip limit must take effect immediately");
    for (int t = 0; t < 20; ++t) {
        simulation.step();
        require(simulation.getTips().size() <= 1 && simulation.getBalanceError() < 1e-5,
                "The reduced tip cap must persist through branching and food-site restarts");
    }
    bool rejected = false;
    try { simulation.setTipLimit(0); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected && simulation.getSettings().maxTips == 1, "Invalid live tip limits must not change the current setting");
}

void checkDeterminismAndBudgets() {
    Simulation a, b;
    Simulation::Settings settings;
    settings.maxNodes = 90;
    a.reset(settings, 42); b.reset(settings, 42);
    for (unsigned turn = 0; turn < 100; ++turn) {
        require(a.step() && b.step(), "Simulations must not stop at a growth-step limit");
        require(a.getNodes().size() == b.getNodes().size() && a.getEdges().size() == b.getEdges().size(),
                "A seed must reproduce graph topology");
        for (std::size_t i = 0; i < a.getNodes().size(); ++i)
            require(a.getNodes()[i].position == b.getNodes()[i].position &&
                    a.getNodes()[i].pressure == b.getNodes()[i].pressure, "A seed must reproduce geometry and pressure");
        for (std::size_t i = 0; i < a.getEdges().size(); ++i)
            require(a.getEdges()[i].flow == b.getEdges()[i].flow &&
                    a.getEdges()[i].conductivity == b.getEdges()[i].conductivity, "A seed must reproduce tube adaptation");
    }
    require(a.getNodes().size() <= 90, "Node budget must stop additional growth");
    settings.branching = 1.1;
    bool rejected = false;
    try { a.reset(settings, 1); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Invalid growth parameters must be rejected");
}
}

int main() {
    try {
        checkHydraulics();
        checkRouteFairness();
        checkContactSemantics();
        checkSharedForkProtection();
        checkLongGrowthAndFusion();
        checkBackboneSettlesWhileExploring();
        checkRecyclingAndContinuousGrowth();
        checkProtectedNodeBudget();
        checkLiveTipLimit();
        checkDeterminismAndBudgets();
        checkGrowth(false, 2026);
        checkGrowth(true, 2026);
        std::cout << "PASS food colonies, contact semantics, flow-free exploration, hydraulic selection, pruning, and seeds\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}
