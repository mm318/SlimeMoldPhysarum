#pragma once

#include "ofMain.h"
#include <array>
#include <cstdint>

class SlimeSimulation {
public:
    static constexpr unsigned teamCount = 25;
    static constexpr unsigned sitesPerTeam = 3;
    static constexpr unsigned agentsPerTeam = 1536;
    static constexpr unsigned historyCapacity = 256;

    struct Agent {
        glm::vec4 position; // x, y, heading (radians), trip distance
        glm::uvec4 identity; // team, last visited site, previously visited site, seed
        glm::vec4 state; // phase (0 explore, 1 return), site visits, action, completed cycles
        glm::uvec4 journey; // home site, breadcrumb count, outbound actions, reserved
    };

    struct TeamStats {
        unsigned spent = 0;
        unsigned exploring = 0;
        unsigned encroaching = 0;
        unsigned attacking = 0;
        unsigned wins = 0;
        unsigned arrivals = 0;
        unsigned defended = 0;
        unsigned defenseFailed = 0;
        unsigned defenseDeclined = 0;
        unsigned outbound = 0;
        unsigned returning = 0;
        unsigned homeReturns = 0;
        unsigned shortcuts = 0;
        unsigned padding[3] = {};
    };

    struct Parameters {
        unsigned budget = 768;
        unsigned exploreCost = 1;
        unsigned encroachCost = 3;
        unsigned attackCost = 12;
        unsigned defenseCost = 8;
        unsigned explorationSteps = 96;
        float defenseProbability = 0.5f;
        float speed = 0.65f;
        float sensorDistance = 3.0f;
        float turnAngle = 0.55f;
        float scentAttraction = 1.4f;
        float freshSpace = 0.8f;
        float trailFollowing = 0.8f;
        float avoidance = 0.65f;
        float exploration = 0.45f;
        float diffusion = 0.12f;
        float evaporation = 0.006f;
        float routeThreshold = 3.0f;
    };

    void setup(int gridWidth = 320, int gridHeight = 256, unsigned seed = 1);
    void reset(unsigned seed);
    void step(const Parameters& parameters);
    void draw(float width, float height, float routeThreshold, bool showAgents,
              bool showPheromones = true, bool showRoutes = true);

    std::array<TeamStats, teamCount> readStats();
    std::vector<Agent> readAgents();
    // xyz: locally diffusing scent from each colony; w: traffic reinforcement.
    std::vector<glm::vec4> readTrails();
    // Also used by integration tests to construct controlled encounters.
    void uploadAgents(const std::vector<Agent>& agents);
    void uploadTrails(const std::vector<glm::vec4>& trails);
    void uploadSites(const std::array<glm::vec4, teamCount * sitesPerTeam>& positions);

    const std::array<glm::vec4, teamCount * sitesPerTeam>& getSites() const { return sites; }
    const std::array<unsigned, teamCount>& getSiteCounts() const { return siteCounts; }
    const std::array<glm::vec4, teamCount>& getColors() const { return colors; }
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    unsigned getTurn() const { return turn; }
    std::size_t fieldIndex(int x, int y, unsigned team) const;

private:
    void bindBuffers();
    static void clearBuffer(ofBufferObject& buffer);

    int width = 0;
    int height = 0;
    unsigned turn = 0;
    int current = 0;
    std::array<glm::vec4, teamCount * sitesPerTeam> sites{};
    std::array<unsigned, teamCount> siteCounts{};
    std::array<glm::vec4, teamCount> colors{};
    ofBufferObject agentBuffer;
    ofBufferObject trailBuffers[2];
    ofBufferObject contributionBuffer;
    ofBufferObject attackBuffer;
    ofBufferObject siteBuffer;
    ofBufferObject statsBuffer;
    ofBufferObject planBuffer;
    ofBufferObject historyBuffer;
    ofShader movementShader;
    ofShader combatShader;
    ofShader applyShader;
    ofShader trailShader;
    ofShader displayShader;
    ofShader agentShader;
    ofVboMesh agentMesh;
};
