#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <vector>

// Li et al. (2020), DOI: 10.1016/j.future.2020.03.055, equations 2.1–2.7.
// A search agent is a complete candidate solution, not a walking particle.
class SmaOptimizer {
public:
    using Position = std::vector<double>;
    using Objective = std::function<double(const Position&)>;
    struct Bound { double lower, upper; };
    struct Settings {
        std::size_t population = 80;
        unsigned iterations = 600;
        double restartProbability = 0.03;
    };

    void reset(const std::vector<Bound>& bounds, Objective objective,
               const Settings& settings, std::uint32_t seed);
    // One complete position update and evaluation. Returns false once finished.
    bool step();
    bool finished() const { return iteration >= settings.iterations; }
    unsigned getIteration() const { return iteration; }
    const Settings& getSettings() const { return settings; }
    const std::vector<Position>& getPopulation() const { return population; }
    const std::vector<double>& getFitness() const { return fitness; }
    const Position& getBestPosition() const { return bestPosition; }
    double getBestFitness() const { return bestFitness; }
    // Includes the initial population at index zero.
    const std::vector<double>& getHistory() const { return history; }

private:
    double uniform();
    void evaluate();
    Settings settings;
    std::vector<Bound> bounds;
    Objective objective;
    std::mt19937 random;
    std::vector<Position> population;
    std::vector<double> fitness, history;
    Position bestPosition;
    double bestFitness = std::numeric_limits<double>::infinity();
    unsigned iteration = 0;
};
