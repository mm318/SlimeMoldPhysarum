#include "ofApp.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace {
ofColor slimeColor(unsigned slime) {
    static const std::array<unsigned, 10> palette{{0x66c2ff, 0xff9671, 0xb59aff, 0x8cdb78, 0xffd166,
                                                 0xf48fb1, 0x51ddd0, 0xc9cf6c, 0xe082ed, 0xbfa38a}};
    return ofColor::fromHex(palette[slime % palette.size()]);
}
}

void ofApp::setup() {
    ofSetWindowTitle("Physarum: growing flow network");
    ofSetFrameRate(60);
    ofEnableAlphaBlending();
    ofxBaseGui::setDefaultWidth(guiWidth - 20);
    controls.setName("Ten competing slimes");
    controls.add(paused.set("Paused [Space]", false));
    controls.add(selectedSlime.set("Inspect slime", 1, 1, PhysarumSimulation::slimeCount));
    controls.add(showExploration.set("Show explored branches", true));
    controls.add(showNodes.set("Show graph nodes", true));
    controls.add(showTips.set("Show growing tips", true));
    controls.add(showFlow.set("Show transport flow", true));
    controls.add(stepsPerFrame.set("Steps per frame", 1, 1, 8));
    controls.add(nodeLimit.set("Live node budget", 5000, 300, 10000));
    controls.add(tipLimit.set("Max tips per slime", 18, 1, 128));
    controls.add(spawnRate.set("New tips / step / slime", 1.0f, 0.0f, 4.0f));
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
    s.spawnRate = value(spawnRate); s.foodBias = value(foodBias);
    s.adaptation = value(adaptation); s.reinforcement = value(reinforcement);
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
        s.spawnRate != current.spawnRate || s.foodBias != current.foodBias ||
        s.adaptation != current.adaptation || s.reinforcement != current.reinforcement)
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
    ofDrawBitmapString("TEN SLIMES / COMPETING FLOW NETWORKS", 24, 28);
    ofSetColor(133, 153, 127);
    ofDrawBitmapString("Own branches join. Foreign contact ends a tip. Opposing tips annihilate.", 24, 50);
    ofFill();
    ofSetColor(10, 21, 14);
    ofDrawRectangle(left, top, side, side);
    const auto& nodes = simulation.getNodes();
    const auto& edges = simulation.getEdges();
    const auto count = simulation.getSlimes().size();
    std::vector<ofMesh> explored(count), scouts(count);
    std::vector<std::array<ofMesh, 9>> tubes(count);
    for (std::size_t s = 0; s < count; ++s) {
        explored[s].setMode(OF_PRIMITIVE_LINES); scouts[s].setMode(OF_PRIMITIVE_LINES);
        for (auto& mesh : tubes[s]) mesh.setMode(OF_PRIMITIVE_LINES);
    }
    for (const auto& edge : edges) {
        const auto a = screen(nodes[edge.a].position), b = screen(nodes[edge.b].position);
        const auto slime = nodes[edge.a].slime;
        if (showExploration) { explored[slime].addVertex(a); explored[slime].addVertex(b); }
        if (!edge.active) continue;
        if (edge.foodLoad < 0.01) {
            if (showExploration) { scouts[slime].addVertex(a); scouts[slime].addVertex(b); }
            continue;
        }
        const double thickness = std::clamp(std::sqrt(edge.conductivity / 1.1), 0.0, 1.0);
        auto& mesh = tubes[slime][std::min(8, int(thickness * 8))];
        mesh.addVertex(a); mesh.addVertex(b);
    }
    if (showExploration) for (const auto& edge : simulation.getRetiredEdges()) {
        explored[edge.slime].addVertex(screen(edge.a)); explored[edge.slime].addVertex(screen(edge.b));
    }
    for (std::size_t s = 0; s < count; ++s) {
        auto color = slimeColor(s); color.a = 35;
        ofSetLineWidth(0.7f); ofSetColor(color); explored[s].draw();
        color.a = 130;
        ofSetLineWidth(0.65f); ofSetColor(color); scouts[s].draw();
    }
    if (showNodes) {
        ofMesh points; points.setMode(OF_PRIMITIVE_POINTS);
        for (const auto& node : nodes) {
            points.addVertex(screen(node.position));
            auto color = slimeColor(node.slime); color.a = 120;
            points.addColor(color);
        }
        ofSetColor(255);
        glPointSize(1.4f); points.draw();
    }
    // Draw each conductance band as one mesh to keep thousands of short tubes
    // inexpensive. Width is quantized only for drawing; physics stays continuous.
    for (std::size_t s = 0; s < count; ++s) for (int band = 0; band <= 8; ++band) {
        const float strength = band / 8.0f;
        const float thickness = 0.45f + 5.0f * strength;
        auto color = slimeColor(s); color.a = 12 + 16 * strength;
        ofSetColor(color);
        ofSetLineWidth(thickness + 3);
        tubes[s][band].draw();
        color = slimeColor(s).getLerped(ofColor::white, 0.35f * strength);
        color.a = 75 + 180 * strength; ofSetColor(color);
        ofSetLineWidth(thickness);
        tubes[s][band].draw();
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
            flow.addColor(slimeColor(nodes[edge.a].slime).getLerped(ofColor::white, 0.75f));
        }
        ofSetColor(255);
        glPointSize(2); flow.draw();
    }
    if (showTips) {
        ofMesh points; points.setMode(OF_PRIMITIVE_POINTS);
        for (const auto& tip : simulation.getTips()) if (tip.active) {
            points.addVertex(screen(nodes[tip.node].position));
            points.addColor(slimeColor(nodes[tip.node].slime).getLerped(ofColor::white, 0.6f));
        }
        ofSetColor(255);
        glPointSize(3.2f); points.draw();
    }
    glPointSize(1);
    for (std::size_t i = 0; i < simulation.getFoods().size(); ++i) {
        const auto& food = simulation.getFoods()[i];
        const auto p = screen(food.position);
        auto color = slimeColor(food.slime); color.a = 35;
        ofFill(); ofSetColor(color);
        ofDrawCircle(p, 12);
        ofNoFill(); ofSetLineWidth(1);
        ofSetColor(slimeColor(food.slime));
        ofDrawCircle(p, 7);
        ofFill(); ofDrawCircle(p, 3);
        const auto& sources = simulation.getSlimes()[food.slime].foods;
        const auto letter = std::find(sources.begin(), sources.end(), i) - sources.begin();
        ofDrawBitmapString(std::to_string(food.slime + 1) + char('A' + letter), p.x - 4, p.y - 18);
    }
    ofSetColor(143, 164, 131);
    ofDrawBitmapString("Color: slime identity   Thick: transport   Thin: scouts   Faint: history", 24, top + side + 25);
    ofDrawBitmapString("Each slime owns 2 or 3 foods. Growth advances at most 0.01 per step.", 24, top + side + 43);
    ofSetColor(17, 25, 19);
    ofDrawRectangle(width, 0, guiWidth, height);
    gui.setPosition(width + 10, 10);
    gui.draw();
    const float textTop = gui.getHeight() + 30;
    const bool growing = nodes.size() < std::size_t(nodeLimit.get());
    const unsigned selected = selectedSlime.get() - 1;
    const auto& sources = simulation.getSlimes()[selected].foods;
    const double path = simulation.getConnectionLength(true, selected);
    std::ostringstream text;
    text << (paused ? "Paused" : growing ? "Exploring / adapting" : "Node budget full / adapting")
         << "\nStep " << simulation.getIteration()
         << "\nSlimes joined: " << simulation.getJoinedSlimes() << " / " << count
         << "\nLive nodes: " << nodes.size() << " / " << nodeLimit
         << "\nActive tubes: " << simulation.getActiveEdges()
         << "\nGrowing tips: " << simulation.getTips().size()
         << std::fixed << std::setprecision(4)
         << "\n\nSlime " << selected + 1 << ": " << sources.size() << " foods"
         << "\nFood groups: " << simulation.getFoodGroups(selected)
         << "\nA-B backbone: " << (std::isfinite(path) ? ofToString(path, 4) : "not established")
         << "\nGrowth cap: 0.0100"
         << "\nSeed: " << seed
         << "\n\nSpace: pause  N: step"
         << "\nR: new seed  Enter: repeat"
         << "\nS / L: save / load controls";
    ofSetColor(216, 226, 199);
    ofDrawBitmapString(text.str(), width + 12, textTop);
    for (std::size_t s = 0; s < count; ++s) {
        const float x = width + 12 + (s % 2) * 130, y = textTop + 265 + (s / 2) * 16;
        ofSetColor(slimeColor(s)); ofDrawRectangle(x, y - 8, 7, 7);
        ofDrawBitmapString(std::to_string(s + 1) + ": " + std::to_string(simulation.getSlimes()[s].foods.size()) + " foods", x + 12, y);
    }
    const float chartTop = textTop + 385, chartWidth = guiWidth - 36, chartHeight = 65;
    if (chartTop + chartHeight + 25 < height) {
        ofSetColor(133, 153, 127);
        ofDrawBitmapString("Selected slime: A-B gap", width + 12, chartTop - 12);
        ofSetColor(29, 42, 30); ofDrawRectangle(width + 12, chartTop, chartWidth, chartHeight);
        ofPolyline curve;
        const auto& history = simulation.getHistory(selected);
        const auto span = PhysarumSimulation::distance(simulation.getFoods()[sources[0]].position,
                                                     simulation.getFoods()[sources[1]].position);
        for (std::size_t i = 0; i < history.size(); ++i)
            curve.addVertex(width + 12 + chartWidth * i / std::max<std::size_t>(1, history.size() - 1),
                            chartTop + chartHeight * (1 - history[i] / span));
        ofSetColor(slimeColor(selected)); ofSetLineWidth(1); curve.draw();
    }
    ofPopStyle();
}
