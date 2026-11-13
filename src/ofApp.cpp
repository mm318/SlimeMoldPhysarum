#include "ofApp.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

void ofApp::setup() {
    ofSetWindowTitle("Slime mould algorithm: 2D path optimization");
    ofSetFrameRate(60);
    ofEnableAlphaBlending();
    ofxBaseGui::setDefaultWidth(guiWidth - 20);
    controls.setName("SMA path search");
    controls.add(paused.set("Paused [Space]", false));
    controls.add(showCandidates.set("Show candidate routes", true));
    controls.add(withObstacles.set("Obstacles [O]", true));
    controls.add(iterationsPerFrame.set("Iterations per frame", 2, 1, 20));
    controls.add(populationSize.set("Search agents", 80, 10, 300));
    controls.add(iterationLimit.set("Iteration limit", 600, 50, 3000));
    controls.add(restartProbability.set("Random restart (z)", 0.03f, 0.0f, 0.3f));
    gui.setup(controls, "sma-settings.xml");
    reset();
}

void ofApp::update() {
    const auto& settings = optimizer.getSettings();
    if (settings.population != static_cast<std::size_t>(populationSize.get()) ||
        settings.iterations != static_cast<unsigned>(iterationLimit.get()) ||
        settings.restartProbability != static_cast<double>(restartProbability.get()) ||
        previousObstacles != withObstacles.get()) reset();
    if (!paused || singleStep) {
        const int count = singleStep ? 1 : iterationsPerFrame.get();
        for (int i = 0; i < count; ++i) {
            if (!optimizer.step()) break;
        }
        singleStep = false;
    }
}

void ofApp::keyPressed(int key) {
    if (key == 'r' || key == 'R') {
        seed = static_cast<unsigned>(ofGetSystemTimeMicros());
        reset();
    }
    else if (key == OF_KEY_RETURN) reset();
    else if (key == 'o' || key == 'O') withObstacles = !withObstacles;
    else if (key == ' ') paused = !paused;
    else if (key == 'n' || key == 'N') { paused = true; singleStep = true; }
    else if (key == 's' || key == 'S') gui.saveToFile("sma-settings.xml");
    else if (key == 'l' || key == 'L') gui.loadFromFile("sma-settings.xml");
}

void ofApp::reset() {
    singleStep = false;
    pathProblem = PathProblem{};
    if (!withObstacles) pathProblem.obstacles.clear();
    previousObstacles = withObstacles;
    SmaOptimizer::Settings settings;
    settings.population = populationSize;
    settings.iterations = iterationLimit;
    settings.restartProbability = restartProbability;
    optimizer.reset(pathProblem.bounds(), [this](const auto& position) {
        return pathProblem.evaluate(position).fitness();
    }, settings, seed);
}

void ofApp::draw() {
    const float width = std::max(1, ofGetWidth() - guiWidth);
    const float height = std::max(1, ofGetHeight());
    ofPushStyle();
    ofBackground(8, 13, 19);
    ofSetColor(227, 237, 231);
    ofDrawBitmapString("SLIME MOULD ALGORITHM / 2D PATH SEARCH", 24, 28);
    ofSetColor(136, 157, 164);
    ofDrawBitmapString("Each search agent proposes a complete route from A to B.", 24, 50);

    const float side = std::max(1.0f, std::min(width - 48, height - 132));
    const float left = (width - side) * 0.5f, top = 76;
    auto screen = [=](PathProblem::Point point) {
        return glm::vec2(left + side * point.x, top + side * point.y);
    };
    ofFill();
    ofSetColor(13, 23, 29);
    ofDrawRectangle(left, top, side, side);
    ofSetLineWidth(1);
    ofSetColor(25, 38, 44);
    for (int i = 0; i <= 10; ++i) {
        const float offset = side * i / 10;
        ofDrawLine(left + offset, top, left + offset, top + side);
        ofDrawLine(left, top + offset, left + side, top + offset);
    }
    for (const auto& obstacle : pathProblem.obstacles) {
        const auto center = screen(obstacle.center);
        ofFill();
        ofSetColor(62, 43, 47);
        ofDrawCircle(center, side * obstacle.radius);
        ofNoFill();
        ofSetColor(156, 94, 95);
        ofDrawCircle(center, side * obstacle.radius);
        ofSetColor(91, 64, 66);
        ofDrawCircle(center, side * (obstacle.radius + PathProblem::clearance));
    }
    auto drawRoute = [&](const SmaOptimizer::Position& position) {
        ofPolyline line;
        for (const auto& point : pathProblem.decode(position)) {
            const auto vertex = screen(point);
            line.addVertex(vertex.x, vertex.y);
        }
        line.draw();
    };
    if (showCandidates) {
        ofSetLineWidth(1);
        ofSetColor(80, 160, 157, 36);
        for (const auto& candidate : optimizer.getPopulation()) drawRoute(candidate);
    }
    const auto evaluation = pathProblem.evaluate(optimizer.getBestPosition());
    ofSetLineWidth(4);
    ofSetColor(evaluation.feasible ? ofColor(208, 240, 103) : ofColor(246, 164, 80));
    drawRoute(optimizer.getBestPosition());
    ofFill();
    for (const auto& point : pathProblem.decode(optimizer.getBestPosition()))
        ofDrawCircle(screen(point), 3);
    ofSetLineWidth(1);
    for (int i = 0; i < 2; ++i) {
        const auto point = screen(i == 0 ? pathProblem.start : pathProblem.goal);
        ofSetColor(8, 13, 19);
        ofDrawCircle(point, 12);
        ofNoFill();
        ofSetColor(208, 240, 103);
        ofDrawCircle(point, 10);
        ofFill();
        ofDrawBitmapString(i == 0 ? "A" : "B", point.x - 4, point.y + 4);
    }
    ofSetColor(166, 187, 189);
    ofDrawBitmapString("Faint: candidate routes   Bright: best route found", 24, top + side + 25);
    ofDrawBitmapString("Disks: obstacles   Outer rings: required clearance", 24, top + side + 43);

    ofSetColor(19, 26, 33);
    ofDrawRectangle(width, 0, guiWidth, height);
    gui.setPosition(width + 10, 10);
    gui.draw();
    const float textTop = gui.getHeight() + 34;
    std::ostringstream text;
    text << (optimizer.finished() ? "Search complete" : paused ? "Paused" : "Optimizing...")
         << "\nIteration " << optimizer.getIteration() << " / " << iterationLimit
         << "\n\nBest route: " << (evaluation.feasible ? "FEASIBLE" : "COLLIDING")
         << std::fixed << std::setprecision(5)
         << "\nLength: " << evaluation.length << "\nScore: " << evaluation.fitness()
         << "\nDirect lower bound: 0.84000"
         << "\n\n8 waypoints / search agent"
         << "\nShorter + collision-free wins"
         << "\nSeed: " << seed
         << "\n\nSpace: pause  N: one iteration"
         << "\nR: new seed  Enter: repeat"
         << "\nO: obstacles on / off"
         << "\nS / L: save / load controls";
    ofSetColor(220, 229, 232);
    ofDrawBitmapString(text.str(), width + 12, textTop);

    const float chartTop = textTop + 285, chartHeight = 95, chartWidth = guiWidth - 36;
    if (chartTop + chartHeight + 52 < height) {
        ofSetColor(136, 157, 164);
        ofDrawBitmapString("Best score (log scale)", width + 12, chartTop - 12);
        ofSetColor(32, 45, 50);
        ofDrawRectangle(width + 12, chartTop, chartWidth, chartHeight);
        const auto& history = optimizer.getHistory();
        const double high = std::log1p(history.front());
        const double low = std::log1p(0.84);
        ofPolyline curve;
        for (std::size_t i = 0; i < history.size(); ++i) {
            const double fraction = (std::log1p(history[i]) - low) / std::max(1e-12, high - low);
            curve.addVertex(width + 12 + chartWidth * i / iterationLimit.get(),
                            chartTop + chartHeight * (1 - std::clamp(fraction, 0.0, 1.0)));
        }
        ofSetColor(208, 240, 103);
        curve.draw();
        ofSetColor(136, 157, 164);
        ofDrawBitmapString("Best found; global optimality\nis not guaranteed by SMA.", width + 12, chartTop + chartHeight + 22);
    }
    ofPopStyle();
}
