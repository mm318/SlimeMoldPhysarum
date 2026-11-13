#include "SmaOptimizer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using Simulation = SmaOptimizer;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void checkPropagation(unsigned seed, double resetProbability) {
    Simulation simulation, repeat;
    Simulation::Settings settings;
    settings.restartProbability = resetProbability;
    simulation.reset(settings, seed);
    repeat.reset(settings, seed);
    for (const auto& agent : simulation.getAgents()) {
        require(agent.position == Simulation::start, "Every agent must start at the exact origin");
        require(agent.history.size() == 1 && agent.history.front() == Simulation::start,
                "Every history must begin at the origin");
    }
    double largestMove = 0, greatestSpread = 0;
    unsigned firstArrival = 0;
    while (!simulation.finished()) {
        std::vector<Simulation::Position> previous;
        std::vector<std::size_t> sizes;
        for (const auto& agent : simulation.getAgents()) {
            previous.push_back(agent.position);
            sizes.push_back(agent.history.size());
        }
        require(simulation.step() && repeat.step(), "Simulation stopped early");
        const auto& agents = simulation.getAgents();
        for (std::size_t i = 0; i < agents.size(); ++i) {
            const auto& agent = agents[i];
            const auto point = agent.position;
            require(std::isfinite(point.x) && std::isfinite(point.y) && point.x >= 0 && point.x <= 1 &&
                    point.y >= 0 && point.y <= 1, "Agent must stay on the unit-square surface");
            const double movement = Simulation::distance(previous[i], point);
            largestMove = std::max(largestMove, movement);
            require(movement <= Simulation::maxStep + 1e-12,
                    "Euclidean movement must never exceed 0.01, including direction resets and arrivals");
            require(point == repeat.getAgents()[i].position, "Equal seeds must reproduce every movement");
            require(agent.history.back() == point && agent.history.front() == Simulation::start,
                    "History must record the actual connected trail from the origin");
            require(agent.history.size() == sizes[i] + (previous[i] == Simulation::goal ? 0 : 1),
                    "Each moving tip records one point per step; arrived tips remain still");
            greatestSpread = std::max(greatestSpread, std::abs(point.y - Simulation::start.y));
            if (agent.arrived) require(point == Simulation::goal, "Arrival must mean reaching the exact goal");
        }
        if (!firstArrival && simulation.getArrivals()) firstArrival = simulation.getIteration();
        require(simulation.getHistory().back() >= 0, "Distance history must be nonnegative");
    }
    require(!simulation.step(), "A completed simulation must stop");
    require(firstArrival >= 84, "No agent may span 0.84 units in fewer than 84 capped movements");
    require(greatestSpread > 0.02, "Agents must explore a 2D region rather than follow one identical line");
    require(simulation.getArrivals() > 0, "The default simulation must discover food");
    if (resetProbability == 0.03)
        require(simulation.getArrivals() == settings.population, "All default agents must reach food within the step budget");
    for (const auto& agent : simulation.getAgents()) {
        double length = 0;
        for (std::size_t j = 1; j < agent.history.size(); ++j) {
            const double segment = Simulation::distance(agent.history[j - 1], agent.history[j]);
            require(segment <= 0.01 + 1e-12, "Stored trails must not contain hidden jumps");
            length += segment;
        }
        require(std::abs(length - agent.travelled) < 1e-10, "Travel distance must match the recorded trail");
        if (agent.arrived) require(length >= 0.84 - 1e-12, "Arrival trail cannot be shorter than the direct distance");
    }
    const auto& history = simulation.getHistory();
    require(history.size() == simulation.getIteration() + 1, "Distance history must include the initial state");
    require(std::is_sorted(history.rbegin(), history.rend()), "Best observed distance must never increase");
    require(history.back() == 0, "Food discovery must be recorded as zero distance");
    std::cout << "Seed " << seed << " z=" << resetProbability << ": first arrival=" << firstArrival
              << ", arrived=" << simulation.getArrivals() << '/' << settings.population
              << ", largest step=" << largestMove << ", spread=" << greatestSpread << '\n';

    simulation.reset(settings, seed);
    for (const auto& agent : simulation.getAgents())
        require(agent.position == Simulation::start && agent.history.size() == 1 && agent.travelled == 0 && !agent.arrived,
                "Reset must clear travel history and return every agent to the start");
    require(simulation.getArrivals() == 0 && simulation.getIteration() == 0, "Reset must clear counters");
}

void checkLimits() {
    Simulation simulation;
    Simulation::Settings settings;
    settings.iterations = 1;
    simulation.reset(settings, 1);
    require(simulation.step() && simulation.finished() && !simulation.step(), "Step budget must be respected");
    require(simulation.getArrivals() == 0, "One local step cannot reach distant food");
    for (const auto& agent : simulation.getAgents())
        require(Simulation::distance(agent.position, Simulation::start) <= 0.01 + 1e-12,
                "Initialization and first movement must not scatter agents across the surface");

    for (double invalid : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
        settings.restartProbability = invalid;
        bool rejected = false;
        try { simulation.reset(settings, 1); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Invalid direction-reset probability must be rejected");
    }
    settings = Simulation::Settings{};
    settings.population = 1;
    bool rejected = false;
    try { simulation.reset(settings, 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "A population smaller than two must be rejected");
}
}

int main() {
    try {
        for (unsigned seed : {1u, 42u, 2026u}) checkPropagation(seed, 0.03);
        checkPropagation(2026, 0);
        checkPropagation(2026, 1);
        checkLimits();
        std::cout << "PASS 2D propagation, exact start/goal, capped local motion, real trails, deterministic seeds, and stopping\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
