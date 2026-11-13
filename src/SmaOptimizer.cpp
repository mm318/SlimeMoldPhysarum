#include "SmaOptimizer.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

double SmaOptimizer::uniform() {
    return std::generate_canonical<double, 53>(random);
}

void SmaOptimizer::reset(const std::vector<Bound>& newBounds, Objective newObjective,
                         const Settings& newSettings, std::uint32_t seed) {
    if (newBounds.empty() || !newObjective || newSettings.population < 2 ||
        newSettings.iterations == 0 || !std::isfinite(newSettings.restartProbability) ||
        newSettings.restartProbability < 0 || newSettings.restartProbability > 1)
        throw std::invalid_argument("Invalid SMA settings, bounds, or objective");
    for (const auto& bound : newBounds) {
        if (!std::isfinite(bound.lower) || !std::isfinite(bound.upper) ||
            bound.lower > bound.upper || !std::isfinite(bound.upper - bound.lower))
            throw std::invalid_argument("SMA bounds must be finite and ordered");
    }
    bounds = newBounds;
    objective = std::move(newObjective);
    settings = newSettings;
    random.seed(seed);
    iteration = 0;
    bestFitness = std::numeric_limits<double>::infinity();
    bestPosition.clear();
    history.clear();
    population.assign(settings.population, Position(bounds.size()));
    fitness.resize(settings.population);
    for (auto& position : population)
        for (std::size_t j = 0; j < bounds.size(); ++j)
            position[j] = bounds[j].lower + uniform() * (bounds[j].upper - bounds[j].lower);
    evaluate();
    history.push_back(bestFitness);
}

void SmaOptimizer::evaluate() {
    for (std::size_t i = 0; i < population.size(); ++i) {
        fitness[i] = objective(population[i]);
        if (!std::isfinite(fitness[i]))
            throw std::runtime_error("SMA objective must return finite fitness");
        if (fitness[i] < bestFitness) {
            bestFitness = fitness[i];
            bestPosition = population[i];
        }
    }
}

bool SmaOptimizer::step() {
    if (!objective || finished()) return false;
    std::vector<std::size_t> order(population.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [this](auto a, auto b) { return fitness[a] < fitness[b]; });
    const double best = fitness[order.front()], worst = fitness[order.back()];
    // Normalize before subtracting to avoid overflow. Equal fitness gives W=1.
    const double scale = std::max({1.0, std::abs(best), std::abs(worst)});
    const double spread = worst / scale - best / scale;
    std::vector<Position> weights(population.size(), Position(bounds.size()));
    for (std::size_t rank = 0; rank < order.size(); ++rank) {
        const auto i = order[rank];
        const double ratio = spread > 0 ?
            std::clamp((fitness[i] / scale - best / scale) / spread, 0.0, 1.0) : 0;
        for (auto& weight : weights[i])
            // log10 follows the authors' MATLAB reference implementation.
            weight = 1 + (rank < order.size() / 2 ? 1 : -1) * uniform() * std::log10(1 + ratio);
    }

    // Start at t=1, avoiding atanh(1) at t=0; both amplitudes reach zero at T.
    const double b = 1 - double(iteration + 1) / settings.iterations;
    const double a = std::atanh(b);
    std::uniform_int_distribution<std::size_t> choose(0, population.size() - 1);
    for (std::size_t i = 0; i < population.size(); ++i) {
        if (uniform() < settings.restartProbability) {
            // Independent coordinates implement the paper's uniform bounded restart.
            for (std::size_t j = 0; j < bounds.size(); ++j)
                population[i][j] = bounds[j].lower + uniform() * (bounds[j].upper - bounds[j].lower);
            continue;
        }
        const double p = std::tanh(std::abs(fitness[i] - bestFitness));
        for (std::size_t j = 0; j < bounds.size(); ++j) {
            const double vb = (2 * uniform() - 1) * a;
            const double vc = (2 * uniform() - 1) * b;
            const bool approach = uniform() < p;
            const auto A = choose(random), B = choose(random);
            // In-place population updates match the reference's agent loop.
            const double value = approach ?
                bestPosition[j] + vb * (weights[i][j] * population[A][j] - population[B][j]) :
                vc * population[i][j];
            population[i][j] = std::clamp(value, bounds[j].lower, bounds[j].upper);
        }
    }
    // Evaluate the final update too, so displayed agents and fitness always agree.
    evaluate();
    ++iteration;
    history.push_back(bestFitness);
    return true;
}
