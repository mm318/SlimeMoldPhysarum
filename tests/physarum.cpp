#include "PhysarumSimulation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <stdexcept>

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
        for (auto& edge : edges)
            edge.conductivity = std::max(1e-6, edge.conductivity + 0.035 *
                (std::pow(std::abs(edge.flow), 1.1) - edge.conductivity));
    }
    require(Simulation::solveFlow(nodes, edges, supply) < 1e-8 && edges[0].flow > 0.95 && edges[2].flow < 0.05,
            "Reinforcement must transfer transport to the shorter competing route");
    require(edges[0].conductivity > 10 * edges[2].conductivity, "Useful tubes must become thicker than unused tubes");
    bool rejected = false;
    try { Simulation::solveFlow(nodes, edges, {1, 0, 0, 0}); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Unbalanced hydraulic supply must be rejected");
}

void checkGrowth(bool sixFoods, unsigned seed) {
    Simulation simulation;
    Simulation::Settings settings;
    settings.sixFoods = sixFoods;
    simulation.reset(settings, seed);
    require(simulation.getNodes().size() == 1 && simulation.getNodes()[0].position == Simulation::start &&
            simulation.getEdges().empty(), "Growth must begin with one seed and no precomputed network");
    require(simulation.getFoods().size() == (sixFoods ? 6u : 2u) && simulation.getReachedFoods() == 1,
            "Only food A is initially occupied; food B must remain a real food source");
    unsigned firstGoal = 0;
    bool loop = false, branching = false;
    for (unsigned turn = 0; turn < 1200; ++turn) {
        require(simulation.step(), "Growth must continue until explicitly paused by its caller");
        const auto& nodes = simulation.getNodes();
        const auto& edges = simulation.getEdges();
        require(nodes.size() <= settings.maxNodes && simulation.getTips().size() <= settings.maxTips,
                "Growth must respect its node and frontier budgets");
        require(simulation.getBalanceError() < 1e-5, "Node pressures must produce mass-conserving transport");
        std::vector<unsigned> degree(nodes.size());
        std::vector<double> balance(nodes.size());
        for (const auto& edge : edges) {
            require(edge.a < nodes.size() && edge.b < nodes.size() && edge.a != edge.b,
                    "Tubes need distinct existing endpoints");
            const double length = Simulation::distance(nodes[edge.a].position, nodes[edge.b].position);
            require(std::abs(length - edge.length) < 1e-10 && length > 0 && length <= 0.01 + 1e-12,
                    "Every grown, joined, or split tube must obey the Euclidean 0.01 limit");
            require(std::isfinite(edge.conductivity) && edge.conductivity > 0 && std::isfinite(edge.flow),
                    "Tube conductance and flow must remain finite");
            if (edge.active) {
                ++degree[edge.a]; ++degree[edge.b];
                balance[edge.a] += edge.flow; balance[edge.b] -= edge.flow;
                if (nodes[edge.a].connected) {
                    const double expected = edge.conductivity / edge.length *
                        (nodes[edge.a].pressure - nodes[edge.b].pressure);
                    require(std::abs(edge.flow - expected) < 1e-9, "Reported flow must use the actual tube conductance");
                }
            } else require(edge.flow == 0, "Pruned tubes must not carry transport");
        }
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto p = nodes[i].position;
            require(std::isfinite(p.x) && std::isfinite(p.y) && p.x >= 0 && p.x <= 1 && p.y >= 0 && p.y <= 1,
                    "The graph must remain on the unit square");
            require(Simulation::distance(Simulation::start, p) <= simulation.getIteration() * 0.01 + 1e-10,
                    "A frontier must not jump ahead of physically possible growth from the seed");
            require(std::abs(balance[i] - simulation.getInjections()[i]) < 1e-5,
                    "Every junction must balance the actual edge flows and nutrient demand");
            branching |= degree[i] >= 3;
        }
        loop |= edges.size() >= nodes.size();
        if (simulation.getFoods()[1].node != Simulation::noNode && !firstGoal) firstGoal = simulation.getIteration();
        for (const auto& food : simulation.getFoods()) if (food.node != Simulation::noNode)
            require(food.node < nodes.size() && nodes[food.node].position == food.position && nodes[food.node].connected,
                    "Reached foods must be actual nodes in the conducting network");
    }
    require(firstGoal >= 84, "Food B cannot be reached in fewer than 84 local growth steps");
    require(simulation.getReachedFoods() == simulation.getFoods().size(), "The seeded scene must discover all food sources");
    require(branching && loop, "The growing graph must form branch junctions and reconnecting routes");
    require(!simulation.getRetiredEdges().empty(), "Weak exploratory branches must retire while growth continues");
    require(std::isfinite(simulation.getConnectionLength()) && simulation.getConnectionLength() >= 0.84 - 1e-9,
            "The active graph must retain a real A-B connection");
    if (!sixFoods) require(simulation.getConnectionLength() < 0.9, "The two-food fixture must retain a short transport route");
    // The working graph must contain only live nodes connected to the seed.
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
    require(std::all_of(visited.begin(), visited.end(), [](bool v) { return v; }), "Every historical branch must connect to the seed");
    require(simulation.getHistory().size() == 1201 &&
            std::is_sorted(simulation.getHistory().rbegin(), simulation.getHistory().rend()),
            "Best approach history must include the seed and record monotonic progress");
    std::cout << (sixFoods ? "Six" : "Two") << " foods, seed " << seed << ": reached=" << simulation.getReachedFoods()
              << ", nodes=" << simulation.getNodes().size() << ", active=" << simulation.getActiveEdges()
              << ", first B=" << firstGoal << ", A-B=" << simulation.getConnectionLength() << '\n';
    simulation.reset(settings, seed);
    require(simulation.getNodes().size() == 1 && simulation.getEdges().empty() && simulation.getRetiredEdges().empty() && simulation.getIteration() == 0,
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
    std::cout << "Continuous recycling: " << births << " births with a " << settings.maxNodes << "-node live cap\n";
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
        checkRecyclingAndContinuousGrowth();
        checkLiveTipLimit();
        checkDeterminismAndBudgets();
        checkGrowth(false, 2026);
        checkGrowth(true, 2026);
        std::cout << "PASS growing graph, local extensions, food discovery, Kirchhoff flow, conductance feedback, junctions, pruning, and seeds\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}
