#include "ofApp.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

void ofApp::setup() {
    ofSetWindowTitle("Slime mould: 2D propagation");
    ofSetFrameRate(60);
    ofEnableAlphaBlending();
    ofxBaseGui::setDefaultWidth(guiWidth - 20);
    controls.setName("Slime propagation");
    controls.add(paused.set("Paused [Space]", false));
    controls.add(showTrails.set("Show exploration trails", true));
    controls.add(showAgents.set("Show growing tips", true));
    controls.add(showClosest.set("Highlight closest trail", false));
    controls.add(iterationsPerFrame.set("Steps per frame", 1, 1, 8));
    controls.add(populationSize.set("Slime agents", 240, 20, 800));
    controls.add(iterationLimit.set("Step limit", 1200, 100, 5000));
    controls.add(restartProbability.set("Direction reset (z)", 0.03f, 0.0f, 0.3f));
    gui.setup(controls, "propagation-settings.xml");
    trailPixels.allocate(fieldSize, fieldSize, OF_PIXELS_RGBA);
    trailTexture.allocate(trailPixels);
    trailTexture.setTextureMinMagFilter(GL_LINEAR, GL_LINEAR);
    reset();
}

void ofApp::reset() {
    singleStep = false;
    SmaOptimizer::Settings settings;
    settings.population = populationSize;
    settings.iterations = iterationLimit;
    settings.restartProbability = restartProbability;
    simulation.reset(settings, seed);
    density.assign(fieldSize * fieldSize, 0);
    blur.resize(density.size());
    deposit(SmaOptimizer::start, SmaOptimizer::start);
    trailsDirty = true;
}

void ofApp::update() {
    const auto& settings = simulation.getSettings();
    if (settings.population != static_cast<std::size_t>(populationSize.get()) ||
        settings.iterations != static_cast<unsigned>(iterationLimit.get()) ||
        settings.restartProbability != static_cast<double>(restartProbability.get())) reset();
    if (!paused || singleStep) {
        const int count = singleStep ? 1 : iterationsPerFrame.get();
        for (int i = 0; i < count && !simulation.finished(); ++i) advance();
        singleStep = false;
    }
    if (trailsDirty) updateTrailTexture();
}

void ofApp::advance() {
    std::vector<SmaOptimizer::Position> previous;
    previous.reserve(simulation.getAgents().size());
    for (const auto& agent : simulation.getAgents()) previous.push_back(agent.position);
    if (!simulation.step()) return;
    for (std::size_t i = 0; i < previous.size(); ++i) {
        const auto next = simulation.getAgents()[i].position;
        if (!(previous[i] == next)) deposit(previous[i], next);
    }
    trailsDirty = true;
}

void ofApp::deposit(SmaOptimizer::Position from, SmaOptimizer::Position to) {
    // Stamp only the segment physically travelled in this step. Density never
    // evaporates: abandoned branches remain as an honest exploration record.
    const int samples = std::max(1, int(std::ceil(SmaOptimizer::distance(from, to) * fieldSize * 2)));
    for (int sample = 1; sample <= samples; ++sample) {
        const double t = double(sample) / samples;
        const double x = (from.x + (to.x - from.x) * t) * (fieldSize - 1);
        const double y = (from.y + (to.y - from.y) * t) * (fieldSize - 1);
        for (int dy = -2; dy <= 2; ++dy) for (int dx = -2; dx <= 2; ++dx) {
            const int px = int(x) + dx, py = int(y) + dy;
            if (px < 0 || py < 0 || px >= fieldSize || py >= fieldSize) continue;
            const double squareDistance = (px - x) * (px - x) + (py - y) * (py - y);
            auto& value = density[py * fieldSize + px];
            value = std::min(256.0f, value + float(0.3 * std::exp(-squareDistance / 1.4)));
        }
    }
}

void ofApp::updateTrailTexture() {
    // A small separable blur gives the deposited trails a soft plasmodium body;
    // the unblurred density supplies the brighter veins inside that body.
    constexpr float kernel[] = {1, 4, 6, 4, 1};
    for (int y = 0; y < fieldSize; ++y) for (int x = 0; x < fieldSize; ++x) {
        float sum = 0;
        for (int k = -2; k <= 2; ++k)
            sum += kernel[k + 2] * density[y * fieldSize + std::clamp(x + k, 0, fieldSize - 1)];
        blur[y * fieldSize + x] = sum / 16;
    }
    for (int y = 0; y < fieldSize; ++y) for (int x = 0; x < fieldSize; ++x) {
        float sum = 0;
        for (int k = -2; k <= 2; ++k)
            sum += kernel[k + 2] * blur[std::clamp(y + k, 0, fieldSize - 1) * fieldSize + x];
        const float body = 1 - std::exp(-sum / 16 * 0.22f);
        const float vein = 1 - std::exp(-density[y * fieldSize + x] * 0.12f);
        trailPixels.setColor(x, y, ofColor(48 + 160 * vein, 83 + 135 * vein,
                                          22 + 56 * vein, 245 * std::sqrt(body)));
    }
    trailTexture.loadData(trailPixels);
    trailsDirty = false;
}

void ofApp::keyPressed(int key) {
    if (key == 'r' || key == 'R') {
        seed = static_cast<unsigned>(ofGetSystemTimeMicros());
        reset();
    }
    else if (key == OF_KEY_RETURN) reset();
    else if (key == ' ') paused = !paused;
    else if (key == 'n' || key == 'N') { paused = true; singleStep = true; }
    else if (key == 's' || key == 'S') gui.saveToFile("propagation-settings.xml");
    else if (key == 'l' || key == 'L') gui.loadFromFile("propagation-settings.xml");
}

void ofApp::draw() {
    const float width = std::max(1, ofGetWidth() - guiWidth);
    const float height = std::max(1, ofGetHeight());
    ofPushStyle();
    ofBackground(7, 12, 10);
    ofSetColor(226, 233, 204);
    ofDrawBitmapString("PHYSARUM / PROPAGATION TOWARD FOOD", 24, 28);
    ofSetColor(132, 151, 128);
    ofDrawBitmapString("Living tips explore in 2D. Their trails remember every step.", 24, 50);
    const float side = std::max(1.0f, std::min(width - 48, height - 132));
    const float left = (width - side) * 0.5f, top = 76;
    auto screen = [=](SmaOptimizer::Position point) {
        return glm::vec2(left + side * point.x, top + side * point.y);
    };
    ofFill();
    ofSetColor(12, 21, 16);
    ofDrawRectangle(left, top, side, side);
    ofNoFill();
    ofSetColor(29, 42, 31);
    ofDrawRectangle(left, top, side, side);
    ofFill();
    if (showTrails) {
        ofSetColor(255);
        trailTexture.draw(left, top, side, side);
    }
    if (showClosest) {
        ofPolyline trail;
        for (const auto& point : simulation.getClosestAgent().history) {
            const auto vertex = screen(point);
            trail.addVertex(vertex.x, vertex.y);
        }
        ofSetColor(232, 238, 153, 190);
        ofSetLineWidth(1.5f);
        trail.draw();
    }
    if (showAgents) for (const auto& agent : simulation.getAgents()) {
        if (agent.arrived) continue;
        const auto point = screen(agent.position);
        ofSetColor(191, 222, 93, 24);
        ofDrawCircle(point, 4);
        ofSetColor(224, 239, 142, 180);
        ofDrawCircle(point, 1.2f);
    }
    const auto origin = screen(SmaOptimizer::start), food = screen(SmaOptimizer::goal);
    ofSetColor(168, 197, 72, 30);
    ofDrawCircle(origin, 13);
    ofSetColor(193, 217, 103);
    ofDrawCircle(origin, 5);
    ofSetColor(216, 183, 101, 25);
    ofDrawCircle(food, 19);
    ofSetColor(167, 141, 80);
    ofDrawCircle(food, 9);
    ofSetColor(226, 211, 155);
    ofDrawCircle(food.x - 2, food.y - 2, 6);
    ofSetColor(172, 185, 151);
    ofDrawBitmapString("START", origin.x - 20, origin.y - 24);
    ofDrawBitmapString("FOOD", food.x - 16, food.y - 24);
    ofSetColor(143, 163, 135);
    ofDrawBitmapString("Green: explored surface   Gold: reinforced veins   Light: growing tips", 24, top + side + 25);
    ofDrawBitmapString("All growth starts at (0.08, 0.5). Food is at (0.92, 0.5).", 24, top + side + 43);

    ofSetColor(17, 25, 20);
    ofDrawRectangle(width, 0, guiWidth, height);
    gui.setPosition(width + 10, 10);
    gui.draw();
    const auto& closest = simulation.getClosestAgent();
    const bool allArrived = simulation.getArrivals() == simulation.getAgents().size();
    const float textTop = gui.getHeight() + 32;
    std::ostringstream text;
    text << (allArrived ? "Food reached by all agents" : simulation.finished() ? "Step limit reached" : paused ? "Paused" : "Propagating...")
         << "\nStep " << simulation.getIteration() << " / " << iterationLimit
         << "\nArrived: " << simulation.getArrivals() << " / " << populationSize
         << std::fixed << std::setprecision(4)
         << "\n\nClosest distance: " << SmaOptimizer::distance(closest.position, SmaOptimizer::goal)
         << (closest.arrived ? "\nShortest arrival trail: " : "\nClosest tip trail: ") << closest.travelled
         << "\nStraight-line distance: 0.8400"
         << "\n\n2 coordinates per agent"
         << "\nMax movement / step: 0.0100"
         << "\nDirection resets stay local"
         << "\nSeed: " << seed
         << "\n\nSpace: pause  N: one step"
         << "\nR: new seed  Enter: repeat"
         << "\nS / L: save / load controls";
    ofSetColor(213, 222, 197);
    ofDrawBitmapString(text.str(), width + 12, textTop);

    const float chartTop = textTop + 260, chartHeight = 95, chartWidth = guiWidth - 36;
    if (chartTop + chartHeight + 60 < height) {
        ofSetColor(132, 151, 128);
        ofDrawBitmapString("Closest approach to food", width + 12, chartTop - 12);
        ofSetColor(29, 42, 31);
        ofDrawRectangle(width + 12, chartTop, chartWidth, chartHeight);
        const auto& history = simulation.getHistory();
        ofPolyline curve;
        for (std::size_t i = 0; i < history.size(); ++i)
            curve.addVertex(width + 12 + chartWidth * i / iterationLimit.get(),
                            chartTop + chartHeight * (1 - history[i] / history.front()));
        ofSetColor(195, 218, 105);
        curve.draw();
        ofSetColor(132, 151, 128);
        ofDrawBitmapString("Local-motion SMA adaptation\nTrails show actual travel.", width + 12, chartTop + chartHeight + 22);
    }
    ofPopStyle();
}
