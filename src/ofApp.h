#pragma once

#include "SlimeSimulation.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void keyPressed(int key) override;

private:
    void reset();
    SlimeSimulation::Parameters parameters() const;

    static constexpr int guiWidth = 280;
    SlimeSimulation simulation;
    ofxPanel gui;
    ofParameterGroup controls;
    ofParameter<int> budget, exploreCost, encroachCost, attackCost, turnsPerFrame;
    ofParameter<float> speed, scentAttraction, freshSpace, sensorDistance, trailFollowing, avoidance, exploration;
    ofParameter<float> diffusion, evaporation, routeThreshold, attackPower, defense;
    ofParameter<bool> paused, showAgents, showPheromones, showRoutes;
    std::array<SlimeSimulation::TeamStats, SlimeSimulation::teamCount> stats{};
    unsigned seed = 0;
    bool singleStep = false;
};
