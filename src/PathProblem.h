#pragma once

#include "SmaOptimizer.h"

// A unit-square 2D surface. Routes advance in x through fixed waypoint stations.
// All geometry and objective calculations are independent of the renderer.
class PathProblem {
public:
    struct Point { double x, y; };
    struct Obstacle { Point center; double radius; };
    struct Evaluation {
        double length = 0;
        double penalty = 0;
        bool feasible = true;
        double fitness() const { return length + penalty; }
    };
    static constexpr std::size_t waypointCount = 8;
    static constexpr double clearance = 0.012;
    Point start{0.08, 0.5}, goal{0.92, 0.5};
    std::vector<Obstacle> obstacles{{{0.5, 0.5}, 0.17}};

    std::vector<SmaOptimizer::Bound> bounds() const;
    std::vector<Point> decode(const SmaOptimizer::Position& position) const;
    Evaluation evaluate(const SmaOptimizer::Position& position) const;
    static double segmentDistance(Point point, Point a, Point b);
};
