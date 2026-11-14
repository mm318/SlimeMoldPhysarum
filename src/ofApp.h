#pragma once

#include "PhysarumSimulation.h"
#include "ofMain.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void keyPressed(int key) override;

private:
    void reset();
    PhysarumSimulation::Settings parameters() const;
    static constexpr int guiWidth = 280;
    PhysarumSimulation simulation;
    ofxPanel gui;
    ofParameterGroup controls;
    ofParameter<int> nodeLimit, tipLimit, stepsPerFrame;
    ofParameter<float> branching, foodBias, adaptation, reinforcement, exploration;
    ofParameter<bool> paused, sixFoods, showExploration, showNodes, showTips, showFlow;
    unsigned seed = 2026;
    double flowClock = 0;
    bool singleStep = false;
};
