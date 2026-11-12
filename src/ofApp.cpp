#include "ofApp.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

void ofApp::setup() {
    ofSetWindowTitle("Slime colonies: 25 teams");
    ofSetFrameRate(60);
    ofEnableAlphaBlending();
    ofBackground(6, 8, 12);
    ofxBaseGui::setDefaultWidth(guiWidth - 20);
    controls.setName("Colony simulation");
    controls.add(paused.set("Paused [Space]", false));
    controls.add(showPheromones.set("Show pheromones", true));
    controls.add(showRoutes.set("Show traffic trails", true));
    controls.add(showAgents.set("Show agents", true));
    controls.add(turnsPerFrame.set("Turns per frame", 1, 1, 8));
    controls.add(budget.set("Team energy per turn", 768, 0, 6144));
    controls.add(exploreCost.set("Explore cost", 1, 1, 8));
    controls.add(encroachCost.set("Foreign scent cost", 3, 2, 24));
    controls.add(attackCost.set("Attack cost", 12, 3, 64));
    controls.add(speed.set("Speed", 0.65f, 0.1f, 1.5f));
    controls.add(scentAttraction.set("Other colony scent", 1.4f, 0.0f, 5.0f));
    controls.add(freshSpace.set("Explore fresh space", 0.8f, 0.0f, 3.0f));
    controls.add(sensorDistance.set("Local sensor distance", 3.0f, 1.0f, 10.0f));
    controls.add(trailFollowing.set("Follow traffic trails", 0.8f, 0.0f, 2.0f));
    controls.add(avoidance.set("Avoid enemy routes", 0.65f, 0.0f, 2.0f));
    controls.add(exploration.set("Exploration turning", 0.45f, 0.0f, 1.0f));
    controls.add(diffusion.set("Pheromone diffusion", 0.12f, 0.0f, 0.5f));
    controls.add(evaporation.set("Pheromone evaporation", 0.006f, 0.0f, 0.05f));
    controls.add(routeThreshold.set("Strong trail density", 3.0f, 0.5f, 24.0f));
    controls.add(attackPower.set("Attack power", 6.0f, 0.1f, 30.0f));
    controls.add(defense.set("Route defense", 1.5f, 0.1f, 6.0f));
    gui.setup(controls, "colony-settings.xml");
    gui.setPosition(ofGetWidth() - guiWidth + 10, 10);
    seed = static_cast<unsigned>(ofGetSystemTimeMicros());
    simulation.setup(320, 256, seed);
}

SlimeSimulation::Parameters ofApp::parameters() const {
    SlimeSimulation::Parameters result;
    result.budget = budget;
    result.exploreCost = exploreCost;
    result.encroachCost = encroachCost;
    result.attackCost = attackCost;
    result.speed = speed;
    result.scentAttraction = scentAttraction;
    result.freshSpace = freshSpace;
    result.sensorDistance = sensorDistance;
    result.trailFollowing = trailFollowing;
    result.avoidance = avoidance;
    result.exploration = exploration;
    result.diffusion = diffusion;
    result.evaporation = evaporation;
    result.routeThreshold = routeThreshold;
    result.attackPower = attackPower;
    result.defense = defense;
    return result;
}

void ofApp::update() {
    // Preserve the requested cost ordering when sliders or saved settings change.
    if (encroachCost <= exploreCost) encroachCost = exploreCost + 1;
    if (attackCost <= encroachCost) attackCost = encroachCost + 1;
    if (!paused || singleStep) {
        int count = singleStep ? 1 : turnsPerFrame.get();
        for (int i = 0; i < count; ++i) simulation.step(parameters());
        singleStep = false;
        if (ofGetFrameNum() % 10 == 0 || paused) stats = simulation.readStats();
    }
}

void ofApp::draw() {
    const float width = std::max(1, ofGetWidth() - guiWidth);
    const float height = std::max(1, ofGetHeight());
    simulation.draw(width, height, routeThreshold, showAgents, showPheromones, showRoutes);
    float scaleX = width / simulation.getWidth();
    float scaleY = height / simulation.getHeight();
    const auto& sites = simulation.getSites();
    const auto& colors = simulation.getColors();
    for (unsigned team = 0; team < SlimeSimulation::teamCount; ++team) {
        const auto& color = colors[team];
        ofSetColor(ofFloatColor(color.r, color.g, color.b));
        for (unsigned site = 0; site < simulation.getSiteCounts()[team]; ++site) {
            glm::vec2 position = glm::vec2(sites[team * 3 + site]) * glm::vec2(scaleX, scaleY);
            ofNoFill();
            ofSetLineWidth(1.5f);
            ofDrawCircle(position.x, position.y, 6);
            ofFill();
            ofDrawCircle(position.x, position.y, 2);
            ofDrawBitmapString(ofToString(team + 1) + char('A' + site), position.x + 8, position.y - 6);
        }
    }
    ofSetLineWidth(1.0f);
    ofSetColor(19, 23, 31);
    ofDrawRectangle(width, 0, guiWidth, height);
    gui.setPosition(width + 10, 10);
    gui.draw();

    unsigned used = 0, active = 0, attacks = 0, wins = 0, arrivals = 0, siteCount = 0;
    for (unsigned team = 0; team < SlimeSimulation::teamCount; ++team) {
        used += stats[team].spent;
        active += stats[team].exploring + stats[team].encroaching + stats[team].attacking;
        attacks += stats[team].attacking;
        wins += stats[team].wins;
        arrivals += stats[team].arrivals;
        siteCount += simulation.getSiteCounts()[team];
    }
    std::ostringstream text;
    text << "25 teams / " << siteCount << " fixed sites\n"
         << SlimeSimulation::agentsPerTeam << " agents/team (fixed)\n"
         << "Turn " << simulation.getTurn() << " / " << std::fixed << std::setprecision(0) << ofGetFrameRate() << " fps\n\n"
         << "Local pheromones only\n"
         << "Mist: pheromones\n"
         << "Colored paths: traffic trails\n"
         << "Bright paths: strong routes\n"
         << "Colored dots: all agents\n"
         << "Rings: fixed sites\n\n"
         << "Latest sampled turn:\n"
         << "Energy: " << used << " / " << budget * 25 << "\n"
         << "Active agents: " << active << "\n"
         << "Attacks / wins: " << attacks << " / " << wins << "\n"
         << "Site arrivals: " << arrivals << "\n\n"
         << "R: new sites + colonies\n"
         << "Space: pause   N: step\n"
         << "S / L: save / load controls\n"
         << "Seed: " << seed;
    ofSetColor(220, 225, 235);
    ofDrawBitmapString(text.str(), width + 12, gui.getHeight() + 32);
}

void ofApp::reset() {
    seed = static_cast<unsigned>(ofGetSystemTimeMicros());
    simulation.reset(seed);
    stats = {};
}

void ofApp::keyPressed(int key) {
    if (key == 'r' || key == 'R') reset();
    else if (key == ' ') paused = !paused;
    else if (key == 'n' || key == 'N') { paused = true; singleStep = true; }
    else if (key == 's' || key == 'S') gui.saveToFile("colony-settings.xml");
    else if (key == 'l' || key == 'L') gui.loadFromFile("colony-settings.xml");
}
