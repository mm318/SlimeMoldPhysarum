#include "ofApp.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

void ofApp::setup() {
    ofSetWindowTitle("Physarum: growing flow network");
    ofSetFrameRate(60);
    ofEnableAlphaBlending();
    ofxBaseGui::setDefaultWidth(guiWidth - 20);
    controls.setName("Growing transport graph");
    controls.add(paused.set("Paused [Space]", false));
    controls.add(sixFoods.set("Video scene: six foods", true));
    controls.add(showExploration.set("Show explored branches", true));
    controls.add(showNodes.set("Show graph nodes", true));
    controls.add(showTips.set("Show growing tips", true));
    controls.add(showFlow.set("Show transport flow", true));
    controls.add(stepsPerFrame.set("Steps per frame", 1, 1, 8));
    controls.add(nodeLimit.set("Live node budget", 5000, 300, 10000));
    controls.add(tipLimit.set("Max growing tips", 18, 1, 128));
    controls.add(branching.set("Branching probability", 0.055f, 0.0f, 0.2f));
    controls.add(foodBias.set("Food growth bias", 0.32f, 0.0f, 0.5f));
    controls.add(adaptation.set("Tube adaptation rate", 0.035f, 0.005f, 0.2f));
    controls.add(reinforcement.set("Flow reinforcement", 1.0f, 0.0f, 2.0f));
    gui.setup(controls, "graph-settings.xml");
    reset();
}

PhysarumSimulation::Settings ofApp::parameters() const {
    PhysarumSimulation::Settings s;
    const auto value = [](float x) { return std::round(double(x) * 1e6) / 1e6; };
    s.maxNodes = nodeLimit; s.maxTips = tipLimit;
    s.branching = value(branching); s.foodBias = value(foodBias);
    s.adaptation = value(adaptation); s.reinforcement = value(reinforcement);
    s.sixFoods = sixFoods;
    return s;
}

void ofApp::reset() {
    singleStep = false;
    flowClock = 0;
    simulation.reset(parameters(), seed);
}

void ofApp::update() {
    const auto s = parameters();
    const auto& current = simulation.getSettings();
    if (s.maxNodes != current.maxNodes ||
        s.branching != current.branching || s.foodBias != current.foodBias ||
        s.adaptation != current.adaptation || s.reinforcement != current.reinforcement || s.sixFoods != current.sixFoods)
        reset();
    else if (s.maxTips != current.maxTips) simulation.setTipLimit(s.maxTips);
    if (!paused || singleStep) {
        const int count = singleStep ? 1 : stepsPerFrame.get();
        for (int i = 0; i < count; ++i) {
            if (!simulation.step()) break;
        }
        singleStep = false;
        flowClock += std::min(0.1, ofGetLastFrameTime());
    }
}

void ofApp::keyPressed(int key) {
    if (key == 'r' || key == 'R') {
        seed = static_cast<unsigned>(ofGetSystemTimeMicros()); reset();
    }
    else if (key == OF_KEY_RETURN) reset();
    else if (key == ' ') paused = !paused;
    else if (key == 'n' || key == 'N') { paused = true; singleStep = true; }
    else if (key == 's' || key == 'S') gui.saveToFile("graph-settings.xml");
    else if (key == 'l' || key == 'L') gui.loadFromFile("graph-settings.xml");
}

void ofApp::draw() {
    const float width = std::max(1, ofGetWidth() - guiWidth), height = std::max(1, ofGetHeight());
    const float side = std::max(1.0f, std::min(width - 48, height - 132));
    const float left = (width - side) * 0.5f, top = 76;
    auto screen = [=](PhysarumSimulation::Position p) { return glm::vec3(left + side * p.x, top + side * p.y, 0); };
    ofPushStyle();
    ofBackground(6, 12, 9);
    ofSetColor(228, 234, 202);
    ofDrawBitmapString("FOOD SOURCES / A GROWING FLOW NETWORK", 24, 28);
    ofSetColor(133, 153, 127);
    ofDrawBitmapString("Branches explore. Flow reinforces useful tubes. Quiet routes fade.", 24, 50);
    ofFill();
    ofSetColor(10, 21, 14);
    ofDrawRectangle(left, top, side, side);
    const auto& nodes = simulation.getNodes();
    const auto& edges = simulation.getEdges();
    ofMesh explored; explored.setMode(OF_PRIMITIVE_LINES);
    ofMesh scouts; scouts.setMode(OF_PRIMITIVE_LINES);
    std::array<ofMesh, 9> tubes;
    for (auto& mesh : tubes) mesh.setMode(OF_PRIMITIVE_LINES);
    for (const auto& edge : edges) {
        const auto a = screen(nodes[edge.a].position), b = screen(nodes[edge.b].position);
        if (showExploration) { explored.addVertex(a); explored.addVertex(b); }
        if (!edge.active) continue;
        if (edge.foodLoad < 0.01) {
            if (showExploration) { scouts.addVertex(a); scouts.addVertex(b); }
            continue;
        }
        const double thickness = std::clamp(std::sqrt(edge.conductivity / 1.1), 0.0, 1.0);
        auto& mesh = tubes[std::min(8, int(thickness * 8))];
        mesh.addVertex(a); mesh.addVertex(b);
    }
    if (showExploration) for (const auto& edge : simulation.getRetiredEdges()) {
        explored.addVertex(screen(edge.a)); explored.addVertex(screen(edge.b));
    }
    ofSetLineWidth(0.7f);
    ofSetColor(71, 96, 66, 72);
    explored.draw();
    ofSetColor(118, 152, 83, 110);
    ofSetLineWidth(0.65f);
    scouts.draw();
    if (showNodes) {
        ofMesh points; points.setMode(OF_PRIMITIVE_POINTS);
        for (const auto& node : nodes) points.addVertex(screen(node.position));
        ofSetColor(130, 153, 115, 120);
        glPointSize(1.4f); points.draw();
    }
    // Draw each conductance band as one mesh to keep thousands of short tubes
    // inexpensive. Width is quantized only for drawing; physics stays continuous.
    for (int band = 0; band <= 8; ++band) {
        const float strength = band / 8.0f;
        const float thickness = 0.45f + 5.0f * strength;
        ofSetColor(222, 205, 53, 12 + 16 * strength);
        ofSetLineWidth(thickness + 3);
        tubes[band].draw();
        ofSetColor(223 + 22 * strength, 205 + 30 * strength, 58 + 55 * strength, 75 + 180 * strength);
        ofSetLineWidth(thickness);
        tubes[band].draw();
    }
    if (showFlow) {
        ofMesh flow; flow.setMode(OF_PRIMITIVE_POINTS);
        for (std::size_t i = 0; i < edges.size(); i += 5) {
            const auto& edge = edges[i];
            if (!edge.active || edge.conductivity < 0.04 || std::abs(edge.flow) < 0.005) continue;
            double phase = std::fmod(flowClock * 1.7 + i * 0.137, 1.0);
            if (edge.flow < 0) phase = 1 - phase;
            const auto a = screen(nodes[edge.a].position), b = screen(nodes[edge.b].position);
            flow.addVertex(a * float(1 - phase) + b * float(phase));
        }
        ofSetColor(246, 248, 184, 220);
        glPointSize(2); flow.draw();
    }
    if (showTips) {
        ofMesh points; points.setMode(OF_PRIMITIVE_POINTS);
        for (const auto& tip : simulation.getTips()) if (tip.active) points.addVertex(screen(nodes[tip.node].position));
        ofSetColor(239, 242, 171);
        glPointSize(3.2f); points.draw();
    }
    glPointSize(1);
    for (std::size_t i = 0; i < simulation.getFoods().size(); ++i) {
        const auto& food = simulation.getFoods()[i];
        const auto p = screen(food.position);
        ofFill(); ofSetColor(209, 185, 111, 35);
        ofDrawCircle(p, 12);
        ofNoFill(); ofSetLineWidth(1);
        ofSetColor(220, 204, 139);
        ofDrawCircle(p, 7);
        ofFill(); ofDrawCircle(p, 3);
        ofDrawBitmapString(std::string(1, char('A' + i)), p.x - 4, p.y - 18);
    }
    ofSetColor(143, 164, 131);
    ofDrawBitmapString("Gold: food transport   Green: scouts   Gray: retired exploration", 24, top + side + 25);
    ofDrawBitmapString("Every branch grows from the existing network, at most 0.01 per step.", 24, top + side + 43);
    ofSetColor(17, 25, 19);
    ofDrawRectangle(width, 0, guiWidth, height);
    gui.setPosition(width + 10, 10);
    gui.draw();
    const float textTop = gui.getHeight() + 30;
    const bool growing = nodes.size() < std::size_t(nodeLimit.get());
    const double path = simulation.getConnectionLength(true);
    std::ostringstream text;
    text << (paused ? "Paused" : growing ? "Exploring / adapting" : "Node budget full / adapting")
         << "\nStep " << simulation.getIteration()
         << "\nFood groups: " << simulation.getFoodGroups() << " (aim: 1)"
         << "\nLive nodes: " << nodes.size() << " / " << nodeLimit
         << "\nActive tubes: " << simulation.getActiveEdges()
         << "\nGrowing tips: " << simulation.getTips().size()
         << std::fixed << std::setprecision(4)
         << "\n\nFood backbone: " << simulation.getTransportLength()
         << "\nAll live tubes: " << simulation.getNetworkLength()
         << "\nA-B backbone: " << (std::isfinite(path) ? ofToString(path, 4) : "not established")
         << "\nGrowth cap: 0.0100"
         << "\nSeed: " << seed
         << "\n\nSpace: pause  N: step"
         << "\nR: new seed  Enter: repeat"
         << "\nS / L: save / load controls";
    ofSetColor(216, 226, 199);
    ofDrawBitmapString(text.str(), width + 12, textTop);
    const float chartTop = textTop + 260, chartWidth = guiWidth - 36, chartHeight = 75;
    if (chartTop + chartHeight + 25 < height) {
        ofSetColor(133, 153, 127);
        ofDrawBitmapString("Recent A-B frontier gap", width + 12, chartTop - 12);
        ofSetColor(29, 42, 30); ofDrawRectangle(width + 12, chartTop, chartWidth, chartHeight);
        ofPolyline curve;
        const auto& history = simulation.getHistory();
        for (std::size_t i = 0; i < history.size(); ++i)
            curve.addVertex(width + 12 + chartWidth * i / std::max<std::size_t>(1, history.size() - 1),
                            chartTop + chartHeight * (1 - history[i] / PhysarumSimulation::distance(PhysarumSimulation::start, PhysarumSimulation::goal)));
        ofSetColor(222, 222, 99); ofSetLineWidth(1); curve.draw();
    }
    ofPopStyle();
}
