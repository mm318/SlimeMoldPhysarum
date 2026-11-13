#include "PathProblem.h"
#include "SmaOptimizer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void checkOptimizer() {
    const auto sphere = [](const auto& x) {
        return (x[0] - 0.37) * (x[0] - 0.37) + (x[1] + 0.4) * (x[1] + 0.4);
    };
    SmaOptimizer first, repeat;
    SmaOptimizer::Settings settings;
    const std::vector<SmaOptimizer::Bound> bounds{{-2, 3}, {-1, 4}};
    first.reset(bounds, sphere, settings, 42);
    repeat.reset(bounds, sphere, settings, 42);
    const double initial = first.getBestFitness();
    for (unsigned i = 0; i < settings.iterations; ++i) {
        require(first.step() && repeat.step(), "SMA stopped early");
        require(first.getPopulation() == repeat.getPopulation(), "Same seed must reproduce the population");
        require(first.getBestFitness() == repeat.getBestFitness(), "Same seed must reproduce convergence");
        const auto& population = first.getPopulation();
        for (std::size_t n = 0; n < population.size(); ++n) {
            for (std::size_t j = 0; j < bounds.size(); ++j)
                require(std::isfinite(population[n][j]) && population[n][j] >= bounds[j].lower &&
                        population[n][j] <= bounds[j].upper, "Agent left its bounds");
            require(sphere(population[n]) == first.getFitness()[n], "Agent fitness must match current position");
        }
    }
    require(first.finished() && !first.step(), "Iteration limit must stop the optimizer");
    require(first.getBestFitness() < initial && first.getBestFitness() < 1e-5,
            "SMA must converge on a shifted sphere, away from the contraction origin");
    require(sphere(first.getBestPosition()) == first.getBestFitness(), "Best position and fitness must agree");
    const auto& history = first.getHistory();
    require(history.size() == settings.iterations + 1, "History must include initialization and final update");
    require(std::is_sorted(history.rbegin(), history.rend()), "Archived best must never get worse");

    // Degenerate rankings and fixed dimensions must not produce NaN weights.
    settings.iterations = 5;
    first.reset({{2, 2}, {-1, 1}}, [](const auto&) { return 7.0; }, settings, 8);
    while (first.step()) {}
    for (const auto& x : first.getPopulation())
        require(x[0] == 2 && std::isfinite(x[1]), "Equal fitness and fixed bounds must remain finite");

    // At the last update a=b=0. With z=0, all positions contract to zero;
    // the last generated population must still be evaluated and archived.
    settings.iterations = 1;
    settings.restartProbability = 0;
    first.reset({{-1, 1}}, [](const auto& x) { return x[0] * x[0]; }, settings, 3);
    first.step();
    require(first.getBestFitness() == 0, "Final position update must be evaluated");

    settings.restartProbability = 1;
    first.reset({{-1, 1}}, [](const auto& x) { return x[0] * x[0]; }, settings, 3);
    first.step();
    require(std::any_of(first.getPopulation().begin(), first.getPopulation().end(),
                       [](const auto& x) { return x[0] != 0; }), "z=1 must use bounded restarts");

    bool rejected = false;
    try { first.reset({{1, -1}}, sphere, settings, 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Invalid bounds must be rejected");
    rejected = false;
    try { first.reset({{-1, 1}}, [](const auto&) { return NAN; }, settings, 1); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "Nonfinite objective values must be reported");
}

void checkPathGeometry() {
    PathProblem problem;
    const SmaOptimizer::Position straight(PathProblem::waypointCount, 0);
    const auto route = problem.decode(straight);
    require(route.front().x == problem.start.x && route.front().y == problem.start.y &&
            route.back().x == problem.goal.x && route.back().y == problem.goal.y,
            "Route endpoints must remain fixed");
    require(!problem.evaluate(straight).feasible, "Straight route must intersect the central obstacle");
    // A tiny obstacle strictly between waypoints detects segment-based checking.
    problem.obstacles = {{{(route[0].x + route[1].x) / 2, 0.5}, 0.002}};
    require(!problem.evaluate(straight).feasible, "Collisions between waypoints must be detected");
    problem.obstacles = {{{0.5, 0.5 + 0.1 + PathProblem::clearance - 0.001}, 0.1}};
    require(!problem.evaluate(straight).feasible, "Route must respect clearance outside obstacle disks");
    problem.obstacles.clear();
    const auto empty = problem.evaluate(straight);
    require(empty.feasible && std::abs(empty.length - 0.84) < 1e-12 && empty.penalty == 0,
            "Empty surface must have a known straight-line optimum");
    require(PathProblem::segmentDistance({1, 0}, {0, 0}, {0, 0}) == 1,
            "Zero-length segments must have a well-defined distance");
}

void checkPathSearch() {
    PathProblem problem;
    SmaOptimizer optimizer;
    SmaOptimizer::Settings settings;
    for (unsigned seed : {1u, 42u, 2026u}) {
        optimizer.reset(problem.bounds(), [&](const auto& x) { return problem.evaluate(x).fitness(); }, settings, seed);
        const double initial = optimizer.getBestFitness();
        while (optimizer.step()) {}
        const auto result = problem.evaluate(optimizer.getBestPosition());
        std::cout << "Obstacle route seed " << seed << ": length=" << result.length
                  << " penalty=" << result.penalty << " initial score=" << initial << '\n';
        require(result.feasible, "Seeded SMA search must find a collision-free route");
        // Continuous shortest path: two tangent segments plus the intervening
        // arc of the clearance-inflated disk. Our finite polyline is slightly longer.
        const double radius = problem.obstacles.front().radius + PathProblem::clearance;
        const double distance = 0.42;
        const double optimum = 2 * std::sqrt(distance * distance - radius * radius) +
            radius * (std::acos(-1.0) - 2 * std::acos(radius / distance));
        require(result.length >= optimum - 1e-9 && result.length < 1.01 * optimum &&
                result.fitness() < initial,
                "SMA detour must improve the initial route and be within 1% of the continuous optimum");
    }
    problem.obstacles.clear();
    optimizer.reset(problem.bounds(), [&](const auto& x) { return problem.evaluate(x).fitness(); }, settings, 2026);
    while (optimizer.step()) {}
    require(problem.evaluate(optimizer.getBestPosition()).feasible &&
            std::abs(optimizer.getBestFitness() - 0.84) < 1e-5,
            "SMA must recover the known optimum on an empty surface");
}
}

int main() {
    try {
        checkOptimizer();
        checkPathGeometry();
        checkPathSearch();
        std::cout << "PASS SMA convergence, determinism, bounds, stopping, and 2D path search\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
