#pragma once

#include "SmaOptimizer.h"
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
    void advance();
    void deposit(SmaOptimizer::Position from, SmaOptimizer::Position to);
    void updateTrailTexture();

    static constexpr int guiWidth = 280;
    static constexpr int fieldSize = 384;
    SmaOptimizer simulation;
    ofxPanel gui;
    ofParameterGroup controls;
    ofParameter<int> populationSize, iterationLimit, iterationsPerFrame;
    ofParameter<float> restartProbability;
    ofParameter<bool> paused, showTrails, showAgents, showClosest;
    std::vector<float> density, blur;
    ofPixels trailPixels;
    ofTexture trailTexture;
    bool trailsDirty = true;
    unsigned seed = 2026;
    bool singleStep = false;
};
