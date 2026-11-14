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
    std::size_t peakEdges = 0;
    bool loop = false, branching = false;
    while (simulation.step()) {
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
        peakEdges = std::max(peakEdges, std::size_t(simulation.getActiveEdges()));
        if (simulation.getFoods()[1].node != Simulation::noNode && !firstGoal) firstGoal = simulation.getIteration();
        for (const auto& food : simulation.getFoods()) if (food.node != Simulation::noNode)
            require(nodes[food.node].position == food.position && nodes[food.node].connected,
                    "Reached foods must be actual nodes in the conducting network");
    }
    require(firstGoal >= 84, "Food B cannot be reached in fewer than 84 local growth steps");
    require(simulation.getReachedFoods() == simulation.getFoods().size(), "The seeded scene must discover all food sources");
    require(branching && loop, "The growing graph must form branch junctions and reconnecting routes");
    require(simulation.getActiveEdges() < peakEdges, "Weak exploratory branches must be pruned during adaptation");
    require(std::isfinite(simulation.getConnectionLength()) && simulation.getConnectionLength() >= 0.84 - 1e-9,
            "The active graph must retain a real A-B connection");
    if (!sixFoods) require(simulation.getConnectionLength() < 0.9, "The two-food fixture must retain a short transport route");
    // The entire historical graph, including retired branches, must remain
    // connected to the seed. No remote skeleton is allowed to appear for free.
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
    require(!simulation.step() && simulation.finished(), "The step limit must stop the simulation");
    require(simulation.getHistory().size() == settings.iterations + 1 &&
            std::is_sorted(simulation.getHistory().rbegin(), simulation.getHistory().rend()),
            "Best approach history must include the seed and record monotonic progress");
    std::cout << (sixFoods ? "Six" : "Two") << " foods, seed " << seed << ": reached=" << simulation.getReachedFoods()
              << ", nodes=" << simulation.getNodes().size() << ", active=" << simulation.getActiveEdges()
              << ", first B=" << firstGoal << ", A-B=" << simulation.getConnectionLength() << '\n';
    simulation.reset(settings, seed);
    require(simulation.getNodes().size() == 1 && simulation.getEdges().empty() && simulation.getIteration() == 0,
            "Reset must clear all grown tubes and transport state");
}

void checkDeterminismAndBudgets() {
    Simulation a, b;
    Simulation::Settings settings;
    settings.maxNodes = 90; settings.iterations = 100;
    a.reset(settings, 42); b.reset(settings, 42);
    while (a.step()) {
        require(b.step(), "Repeated simulation stopped early");
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
        checkDeterminismAndBudgets();
        checkGrowth(false, 2026);
        checkGrowth(true, 2026);
        std::cout << "PASS growing graph, local extensions, food discovery, Kirchhoff flow, conductance feedback, junctions, pruning, and seeds\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}
