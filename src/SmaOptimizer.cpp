#include "SmaOptimizer.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace {
using Position = SmaOptimizer::Position;
Position add(Position a, Position b) { return {a.x + b.x, a.y + b.y}; }
Position subtract(Position a, Position b) { return {a.x - b.x, a.y - b.y}; }
Position scale(Position value, double amount) { return {value.x * amount, value.y * amount}; }
Position unit(Position value) {
    const double length = std::hypot(value.x, value.y);
    return length > 1e-12 ? scale(value, 1 / length) : Position{};
}
}

double SmaOptimizer::distance(Position a, Position b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double SmaOptimizer::uniform() {
    return std::generate_canonical<double, 53>(random);
}

SmaOptimizer::Position SmaOptimizer::randomDirection() {
    const double angle = uniform() * 2 * std::acos(-1.0);
    return {std::cos(angle), std::sin(angle)};
}

void SmaOptimizer::reset(const Settings& newSettings, std::uint32_t seed) {
    if (newSettings.population < 2 || newSettings.iterations == 0 ||
        !std::isfinite(newSettings.restartProbability) ||
        newSettings.restartProbability < 0 || newSettings.restartProbability > 1)
        throw std::invalid_argument("Invalid SMA propagation settings");
    settings = newSettings;
    random.seed(seed);
    iteration = 0;
    arrivals = closest = 0;
    agents.assign(settings.population, Agent{});
    for (auto& agent : agents) {
        agent.position = start;
        agent.heading = randomDirection();
        agent.history.push_back(start);
    }
    history.assign(1, distance(start, goal));
}

bool SmaOptimizer::step() {
    if (agents.empty() || finished()) return false;
    std::vector<double> fitness(agents.size());
    std::vector<Position> positions;
    positions.reserve(agents.size());
    for (std::size_t i = 0; i < agents.size(); ++i) {
        positions.push_back(agents[i].position);
        fitness[i] = distance(positions.back(), goal);
    }
    std::vector<std::size_t> order(agents.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) { return fitness[a] < fitness[b]; });
    const double best = fitness[order.front()], worst = fitness[order.back()];
    std::vector<Position> weights(agents.size());
    for (std::size_t rank = 0; rank < order.size(); ++rank) {
        const auto i = order[rank];
        const double ratio = worst > best ? (fitness[i] - best) / (worst - best) : 0;
        const double sign = rank < order.size() / 2 ? 1 : -1;
        weights[i] = {1 + sign * uniform() * std::log10(1 + ratio),
                      1 + sign * uniform() * std::log10(1 + ratio)};
    }
    const double b = 1 - double(iteration + 1) / settings.iterations;
    const double a = std::atanh(b);
    std::uniform_int_distribution<std::size_t> choose(0, agents.size() - 1);
    for (std::size_t i = 0; i < agents.size(); ++i) {
        auto& agent = agents[i];
        if (agent.arrived) continue;
        const auto previous = agent.position;
        if (fitness[i] <= maxStep) {
            // Finishing the last short segment obeys the same distance limit.
            agent.position = goal;
            agent.arrived = true;
            ++arrivals;
        } else {
            if (uniform() < settings.restartProbability) agent.heading = randomDirection();
            const double p = std::tanh(std::abs(fitness[i] - best));
            Position oscillation;
            if (uniform() < p) {
                const auto A = subtract(positions[choose(random)], previous);
                const auto B = subtract(positions[choose(random)], previous);
                oscillation = add(subtract(positions[order.front()], previous),
                    {(2 * uniform() - 1) * a * (weights[i].x * A.x - B.x),
                     (2 * uniform() - 1) * a * (weights[i].y * A.y - B.y)});
            } else {
                // Contract heading perturbations, rather than world coordinates.
                oscillation = scale(agent.heading, (2 * uniform() - 1) * b);
            }
            // The obstacle-free distance field supplies a local food-scent
            // direction. Persistence and random turns spread a searching front;
            // the scent bias increases as SMA's oscillations shrink.
            const auto scent = unit(subtract(goal, previous));
            Position steering = add(scale(agent.heading, 1.0), scale(scent, 0.09 + 0.3 * (1 - b)));
            steering = add(steering, scale(unit(oscillation), 0.04));
            steering = add(steering, scale(randomDirection(), 0.12 * b));
            agent.heading = unit(steering);
            const double length = maxStep * (0.65 + 0.35 * uniform());
            agent.position = add(previous, scale(agent.heading, length));
            // Turn back into the surface at its edges instead of pinning a
            // persistent outward heading against a clamped coordinate.
            if (agent.position.x < 0 || agent.position.x > 1) agent.heading.x = -agent.heading.x;
            if (agent.position.y < 0 || agent.position.y > 1) agent.heading.y = -agent.heading.y;
            agent.position.x = std::clamp(agent.position.x, 0.0, 1.0);
            agent.position.y = std::clamp(agent.position.y, 0.0, 1.0);
        }
        agent.travelled += distance(previous, agent.position);
        agent.history.push_back(agent.position);
    }
    ++iteration;
    measure();
    return true;
}

void SmaOptimizer::measure() {
    closest = 0;
    for (std::size_t i = 1; i < agents.size(); ++i) {
        const double candidate = distance(agents[i].position, goal);
        const double current = distance(agents[closest].position, goal);
        if (candidate < current || (candidate == current && agents[i].travelled < agents[closest].travelled)) closest = i;
    }
    history.push_back(std::min(history.back(), distance(agents[closest].position, goal)));
}
