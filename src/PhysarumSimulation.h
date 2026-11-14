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
        bool connected = true;
    };
    struct Edge {
        std::size_t a, b;
        double length;
        double conductivity = 0.003;
        double flow = 0;
        unsigned born = 0;
        bool active = true;
        double foodLoad = 0;
        double foodDrive = 0;
        double explorationDrive = 0;
    };
    struct Food { Position position; std::size_t node = noNode; };
    struct RetiredEdge { Position a, b; };
    struct Tip {
        std::size_t node = 0, previous = noNode, target = noNode;
        Position direction;
        unsigned age = 0, sinceBranch = 0;
        bool active = true;
    };
    struct Settings {
        std::size_t maxNodes = 1800;
        unsigned maxTips = 18;
        double branching = 0.055;
        double foodBias = 0.32;
        double adaptation = 0.035;
        double reinforcement = 1.0;
        double shortening = 0.3;
        bool sixFoods = true;
    };
    static constexpr Position start{0.08, 0.5};
    static constexpr Position goal{0.92, 0.5};
    static constexpr double maxStep = 0.01;
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
    const std::vector<double>& getInjections() const { return injections; }
    const std::deque<double>& getHistory() const { return history; }
    unsigned getReachedFoods() const;
    unsigned getActiveEdges() const;
    double getBalanceError() const { return balanceError; }
    double getNetworkLength() const;
    double getTransportLength() const;
    double getConnectionLength(bool transportOnly = false) const;
    static double distance(Position a, Position b);
    static double flowResponse(double flow, bool multipleFoods);
    // Solve a supplied network too, e.g. for controlled hydraulic experiments.
    // Positive injections supply flow, negative injections consume it. Sum=0.
    // Node zero is the pressure reference. Disconnected, undriven nodes are ignored.
    static double solveFlow(std::vector<Node>& nodes, std::vector<Edge>& edges,
                            const std::vector<double>& injections);

private:
    double uniform();
    Position randomDirection();
    std::size_t chooseTarget(std::size_t node, Position direction, std::size_t avoid = noNode) const;
    bool addEdge(std::size_t a, std::size_t b);
    void grow();
    bool extend(Tip& tip);
    void adapt();
    void retire();
    void shorten();
    void transport();
    void measure();
    Settings settings;
    std::mt19937 random;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<Tip> tips;
    std::vector<Food> foods;
    std::vector<double> injections;
    std::deque<double> history;
    std::deque<RetiredEdge> retired;
    unsigned iteration = 0;
    double balanceError = 0;
};
