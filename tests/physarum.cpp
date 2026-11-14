#include "PhysarumSimulation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>

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
    double low = 10, high = 0;
    for (unsigned turn = 0; turn < 2500; ++turn) {
        simulation.step();
        if (turn > 400) {
            const double route = simulation.getConnectionLength(true);
            require(std::isfinite(route) && route < 0.85,
                    "Continued branching must not displace the efficient food-carrying A-B backbone");
            if (turn > 2000) { low = std::min(low, route); high = std::max(high, route); }
        }
        require(simulation.getBalanceError() < 1e-5, "Long-running adaptation must retain accurate pressure/flow balance");
    }
    require(high - low < 0.002 && !simulation.getTips().empty(),
            "The mature route must settle while exploration remains active");
    std::cout << "Stable two-food backbone during continuous exploration: " << low << ".." << high << '\n';
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
            std::abs(simulation.getEdges()[0].flow - settings.exploration) < 1e-9,
            "A newborn thin feeder must carry its tip's prescribed consumption through solved pressure");
    unsigned joined = 0;
    bool oldFeeder = false, fading = false, otherFoodLaunched = false;
    double previousFade = settings.exploration + 1;
    for (unsigned turn = 1; turn < 180; ++turn) {
        simulation.step();
        if (simulation.getFoodGroups() == 1 && !joined) joined = simulation.getIteration();
        if (!joined && turn > 40)
            for (const auto& edge : simulation.getEdges()) oldFeeder |= turn - edge.born > 31;
        for (const auto& tip : simulation.getTips()) otherFoodLaunched |= tip.food == 1;
        double fade = 0;
        for (const auto& demand : simulation.getGrowthDemands())
            if (demand.fading && demand.food == 0 && demand.node == simulation.getFoods()[1].node) fade += demand.flow;
        if (fade > 0) {
            require(fade < previousFade, "A fused tip's withdrawal must decrease on every fading step");
            require(simulation.getIteration() < joined + Simulation::fusionFadeSteps,
                    "A fused tip must stop consuming exploration flow when its fade finishes");
            previousFade = fade; fading = true;
        }
    }
    require(oldFeeder && joined >= 84 && joined < 120 && fading && otherFoodLaunched,
            "Supported growth must outlive 31 steps, reach the other food, fade, and rotate the single tip slot");
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
    bool loop = false, branching = false, continuedSupport = false, fading = false, lateral = false;
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
            require(edge.explorationDrive <= Simulation::explorationConductivityLimit,
                    "Exploration reinforcement must remain capped at thin-tube conductivity");
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
        std::vector<double> spent(simulation.getFoods().size()), growth(nodes.size());
        for (const auto& demand : simulation.getGrowthDemands()) {
            require(demand.food < spent.size() && demand.node < nodes.size() && demand.flow > 0,
                    "Every tip sink must have a valid supplying food and positive consumption");
            spent[demand.food] += demand.flow;
            growth[demand.node] -= demand.flow;
            growth[simulation.getFoods()[demand.food].node] += demand.flow;
            fading |= demand.fading;
        }
        for (auto flow : spent) require(flow <= settings.exploration + 1e-12,
                                       "Each food's growth budget must be shared across active and fading tips");
        for (std::size_t i = 0; i < nodes.size(); ++i)
            require(std::abs(growth[i] - simulation.getGrowthInjections()[i]) < 1e-12,
                    "A tip must consume real flow supplied at its originating food");
        continuedSupport |= simulation.getFoodGroups() == 1 && !simulation.getGrowthDemands().empty() && turn > 200;
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
    require(continuedSupport && fading && lateral,
            "Exploration must continue after connection, fade on fusion, and sprout from established trunks");
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
        checkRouteFairness();
        checkLongGrowthAndFusion();
        checkBackboneSettlesWhileExploring();
        checkRecyclingAndContinuousGrowth();
        checkLiveTipLimit();
        checkDeterminismAndBudgets();
        checkGrowth(false, 2026);
        checkGrowth(true, 2026);
        std::cout << "PASS food colonies, meeting fronts, growth sinks, hydraulic selection, local extensions, pruning, and seeds\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}
