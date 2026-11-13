#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

// A local-motion adaptation of SMA: agents are physical 2D tips. Ranked
// weights and oscillating proposals steer motion; no update teleports an agent.
class SmaOptimizer {
public:
    struct Position {
        double x = 0, y = 0;
        bool operator==(const Position& other) const { return x == other.x && y == other.y; }
    };
    struct Agent {
        Position position;
        Position heading;
        std::vector<Position> history;
        double travelled = 0;
        bool arrived = false;
    };
    struct Settings {
        std::size_t population = 240;
        unsigned iterations = 1200;
        double restartProbability = 0.03; // Reset heading, never position.
    };
    static constexpr Position start{0.08, 0.5};
    static constexpr Position goal{0.92, 0.5};
    static constexpr double maxStep = 0.01; // Euclidean distance per iteration.

    void reset(const Settings& settings, std::uint32_t seed);
    bool step();
    bool finished() const { return !agents.empty() && (iteration >= settings.iterations || arrivals == agents.size()); }
    unsigned getIteration() const { return iteration; }
    std::size_t getArrivals() const { return arrivals; }
    const Settings& getSettings() const { return settings; }
    const std::vector<Agent>& getAgents() const { return agents; }
    const Agent& getClosestAgent() const { return agents.at(closest); }
    // Best distance ever reached, including the initial state.
    const std::vector<double>& getHistory() const { return history; }
    static double distance(Position a, Position b);

private:
    double uniform();
    Position randomDirection();
    void measure();
    Settings settings;
    std::mt19937 random;
    std::vector<Agent> agents;
    std::vector<double> history;
    std::size_t closest = 0, arrivals = 0;
    unsigned iteration = 0;
};
