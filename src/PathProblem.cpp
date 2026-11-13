#include "PathProblem.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

std::vector<SmaOptimizer::Bound> PathProblem::bounds() const {
    return std::vector<SmaOptimizer::Bound>(waypointCount, {-1, 1});
}

std::vector<PathProblem::Point> PathProblem::decode(const SmaOptimizer::Position& position) const {
    if (position.size() != waypointCount)
        throw std::invalid_argument("Path must contain eight waypoint ordinates");
    std::vector<Point> route{start};
    for (std::size_t j = 0; j < position.size(); ++j) {
        if (!std::isfinite(position[j]) || position[j] < -1 || position[j] > 1)
            throw std::invalid_argument("Path ordinate must be finite and in [-1,1]");
        const double t = double(j + 1) / (waypointCount + 1);
        route.push_back({start.x + t * (goal.x - start.x), (position[j] + 1) * 0.5});
    }
    route.push_back(goal);
    return route;
}

double PathProblem::segmentDistance(Point point, Point a, Point b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double squareLength = dx * dx + dy * dy;
    const double t = squareLength > 0 ? std::clamp(
        ((point.x - a.x) * dx + (point.y - a.y) * dy) / squareLength, 0.0, 1.0) : 0;
    return std::hypot(point.x - a.x - t * dx, point.y - a.y - t * dy);
}

PathProblem::Evaluation PathProblem::evaluate(const SmaOptimizer::Position& position) const {
    const auto route = decode(position);
    Evaluation result;
    for (std::size_t j = 1; j < route.size(); ++j) {
        result.length += std::hypot(route[j].x - route[j - 1].x, route[j].y - route[j - 1].y);
        for (const auto& obstacle : obstacles) {
            const double penetration = obstacle.radius + clearance -
                segmentDistance(obstacle.center, route[j - 1], route[j]);
            if (penetration > 0) {
                result.feasible = false;
                // A fixed violation charge keeps any colliding path worse than a
                // feasible one; depth provides a gradient toward the free surface.
                result.penalty += 100 + 1000 * penetration;
            }
        }
    }
    return result;
}
