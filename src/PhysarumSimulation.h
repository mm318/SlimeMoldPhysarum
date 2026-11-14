#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <vector>

// Growing spatial graph with Kirchhoff flow and adaptive tube conductance.
class PhysarumSimulation {
public:
    struct Position {
        double x = 0, y = 0;
        bool operator==(const Position& other) const { return x == other.x && y == other.y; }
    };
    static constexpr std::size_t noNode = std::numeric_limits<std::size_t>::max();
    struct Node {
        Position position;
        double pressure = 0;
        unsigned born = 0;
        unsigned slime = 0;
    };
    enum class TubeState { Exploring, Established, Abandoned };
    struct Edge {
        std::size_t a, b;
        double length;
        double conductivity = 0.003;
        double flow = 0;
        unsigned born = 0;
        bool active = true;
        double foodLoad = 0;
        unsigned weakSteps = 0;
        TubeState state = TubeState::Established;
    };
    struct Food { Position position; std::size_t node = noNode; unsigned slime = 0; };
    struct RetiredEdge { Position a, b; unsigned slime = 0; };
    struct Slime { std::vector<std::size_t> foods; std::deque<double> history; };
    struct Tip {
        std::size_t node = 0, target = noNode, food = 0;
        Position direction;
        unsigned age = 0, sinceBranch = 0;
        bool active = true;
    };
    struct Settings {
        std::size_t maxNodes = 5000;
        unsigned maxTips = 18;
        double branching = 0.055;
        double foodBias = 0.32;
        double adaptation = 0.035;
        double reinforcement = 1.0;
    };
    static constexpr unsigned slimeCount = 10;
    static constexpr double maxStep = 0.01;
    static constexpr double contactRadius = 0.0035;
    static constexpr unsigned pruningDelay = 24;
    static constexpr std::size_t retiredHistoryLimit = 20000;
    static constexpr std::size_t progressHistoryLimit = 4096;

    void reset(const Settings& settings, std::uint32_t seed);
    void setTipLimit(unsigned limit);
    bool step();
    unsigned getIteration() const { return iteration; }
    const Settings& getSettings() const { return settings; }
    const std::vector<Node>& getNodes() const { return nodes; }
    const std::vector<Edge>& getEdges() const { return edges; }
    const std::deque<RetiredEdge>& getRetiredEdges() const { return retired; }
    const std::vector<Tip>& getTips() const { return tips; }
    const std::vector<Food>& getFoods() const { return foods; }
    const std::vector<Slime>& getSlimes() const { return slimes; }
    const std::vector<double>& getInjections() const { return injections; }
    const std::deque<double>& getHistory(unsigned slime = 0) const { return slimes.at(slime).history; }
    unsigned getFoodGroups(std::size_t slime = noNode) const;
    unsigned getJoinedSlimes() const;
    unsigned getActiveEdges() const;
    double getBalanceError() const { return balanceError; }
    double getNetworkLength() const;
    double getTransportLength() const;
    double getConnectionLength(bool transportOnly = false, unsigned slime = 0) const;
    static double distance(Position a, Position b);
    // Apply the same local tube adaptation to supplied hydraulic experiments.
    static void adaptTubes(std::vector<Edge>& tubes, const Settings& settings, unsigned iteration);
    // Solve a supplied network too, e.g. for controlled hydraulic experiments.
    // Positive injections supply flow, negative injections consume it. Sum=0.
    // Each component has its own pressure reference and must balance its supply.
    static double solveFlow(std::vector<Node>& nodes, std::vector<Edge>& edges,
                            const std::vector<double>& injections);

private:
    friend struct PhysarumSimulationTestAccess;
    double uniform();
    Position randomDirection();
    std::size_t component(std::size_t node) const;
    void rebuildComponents();
    void initializeColonies();
    std::size_t addNode(Position position, double pressure = 0, unsigned slime = 0);
    unsigned countTips(unsigned slime) const;
    void launchFoodTip(std::size_t food, unsigned fan = 1);
    void replenishTips();
    std::size_t chooseTarget(std::size_t node, Position direction, std::size_t avoid = noNode) const;
    bool addEdge(std::size_t a, std::size_t b);
    void grow();
    bool extend(Tip& tip);
    void updateExploration();
    void retire();
    void transport();
    void measure();
    Settings settings;
    std::mt19937 random;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<Tip> tips;
    std::vector<Food> foods;
    std::vector<Slime> slimes;
    std::vector<std::size_t> parents;
    std::vector<double> injections;
    std::deque<RetiredEdge> retired;
    unsigned iteration = 0;
    std::size_t nextFood = 0;
    double balanceError = 0;
};
