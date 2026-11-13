#pragma once

#include "PathProblem.h"
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

    static constexpr int guiWidth = 280;
    PathProblem pathProblem;
    SmaOptimizer optimizer;
    ofxPanel gui;
    ofParameterGroup controls;
    ofParameter<int> populationSize, iterationLimit, iterationsPerFrame;
    ofParameter<float> restartProbability;
    ofParameter<bool> paused, withObstacles, showCandidates;
    bool previousObstacles = true;
    unsigned seed = 2026;
    bool singleStep = false;
};
