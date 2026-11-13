#include "SlimeSimulation.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace {
constexpr float pi = 3.14159265358979323846f;

template<typename T>
std::vector<T> readBuffer(ofBufferObject& buffer, std::size_t count) {
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    auto* data = buffer.map<T>(GL_READ_ONLY);
    if (!data) throw std::runtime_error("Cannot read simulation buffer");
    std::vector<T> result(data, data + count);
    buffer.unmap();
    return result;
}
}

static_assert(sizeof(SlimeSimulation::Agent) == 64, "Agent must match GLSL std430");
static_assert(sizeof(SlimeSimulation::TeamStats) == 64, "TeamStats must match GLSL std430");

void SlimeSimulation::setup(int gridWidth, int gridHeight, unsigned seed) {
    if (gridWidth < 32 || gridHeight < 32) {
        throw std::invalid_argument("Simulation grid must be at least 32 by 32");
    }
    width = gridWidth;
    height = gridHeight;
    if (!movementShader.loadCompute("shadersGL3/particleShader.compute") ||
        !combatShader.loadCompute("shadersGL3/combat.compute") ||
        !applyShader.loadCompute("shadersGL3/applyAgents.compute") ||
        !trailShader.loadCompute("shadersGL3/trailMapShader.compute") ||
        !displayShader.load("shadersGL3/shader") ||
        !agentShader.load("shadersGL3/agents")) {
        throw std::runtime_error("Cannot load simulation shaders");
    }
    // ofShader in OF 0.12.1 doesn't propagate all link failures through load().
    for (auto* shader : {&movementShader, &combatShader, &applyShader, &trailShader, &displayShader, &agentShader}) {
        GLint linked = GL_FALSE;
        glGetProgramiv(shader->getProgram(), GL_LINK_STATUS, &linked);
        if (linked != GL_TRUE) throw std::runtime_error("Simulation shader failed to link");
    }
    for (unsigned team = 0; team < teamCount; ++team) {
        // Golden-ratio hue spacing separates adjacent team IDs visually.
        ofFloatColor color;
        color.setHsb(std::fmod(team * 0.61803398875f, 1.0f), 0.72f, 1.0f);
        colors[team] = glm::vec4(color.r, color.g, color.b, 1.0f);
    }
    agentMesh.clear();
    agentMesh.setMode(OF_PRIMITIVE_POINTS);
    agentMesh.addVertices(std::vector<glm::vec3>(teamCount * agentsPerTeam, glm::vec3(0)));
    reset(seed);
}

void SlimeSimulation::reset(unsigned seed) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    std::uniform_int_distribution<unsigned> countDistribution(2, 3);
    std::vector<Agent> agents;
    agents.reserve(teamCount * agentsPerTeam);
    std::vector<glm::vec2> history(teamCount * agentsPerTeam * historyCapacity, glm::vec2(0));
    const float margin = 5.0f;
    const float separation = std::min(width, height) * 0.20f;

    for (unsigned team = 0; team < teamCount; ++team) {
        unsigned count = siteCounts[team] = countDistribution(random);
        for (unsigned site = 0; site < sitesPerTeam; ++site) {
            glm::vec2 position(0.0f);
            if (site < count) {
                // Sites of the same team start well apart. Other teams may
                // overlap: ownership is determined by agents, not spawn order.
                for (int attempt = 0; attempt < 100; ++attempt) {
                    position = {margin + unit(random) * (width - 2 * margin),
                                margin + unit(random) * (height - 2 * margin)};
                    bool separated = true;
                    for (unsigned previous = 0; previous < site; ++previous) {
                        separated &= glm::distance(position, glm::vec2(sites[team * 3 + previous])) >= separation;
                    }
                    if (separated) break;
                }
            }
            sites[team * sitesPerTeam + site] = glm::vec4(position, site < count ? 1.0f : 0.0f, 0.0f);
        }

        for (unsigned i = 0; i < agentsPerTeam; ++i) {
            unsigned home = i % count;
            // sqrt makes the initial distribution uniform over a tight disk.
            float radius = 1.6f * std::sqrt(unit(random));
            float angle = unit(random) * 2 * pi;
            glm::vec2 position = glm::vec2(sites[team * 3 + home]) +
                                 radius * glm::vec2(std::cos(angle), std::sin(angle));
            // Unbiased headings: scouts have no knowledge of other sites.
            float heading = unit(random) * 2 * pi;
            agents.push_back({glm::vec4(position, heading, 0.0f),
                              glm::uvec4(team, team * 3 + home, team * 3 + home, random()),
                              glm::vec4(0.0f), glm::uvec4(team * 3 + home, 1, 0, 0)});
            history[(agents.size() - 1) * historyCapacity] = glm::vec2(sites[team * 3 + home]);
        }
    }

    agentBuffer.allocate(agents, GL_DYNAMIC_DRAW);
    historyBuffer.allocate(history, GL_DYNAMIC_DRAW);
    planBuffer.allocate(teamCount * agentsPerTeam * (sizeof(glm::vec4) + sizeof(glm::uvec4)), GL_DYNAMIC_DRAW);
    siteBuffer.allocate(sizeof(sites), sites.data(), GL_STATIC_DRAW);
    statsBuffer.allocate(sizeof(TeamStats) * teamCount, GL_DYNAMIC_DRAW);
    const auto fields = static_cast<GLsizeiptr>(width) * height * teamCount;
    for (auto& buffer : trailBuffers) {
        buffer.allocate(fields * sizeof(glm::vec4), GL_DYNAMIC_DRAW);
        clearBuffer(buffer);
    }
    contributionBuffer.allocate(fields * sizeof(glm::uvec4), GL_DYNAMIC_DRAW);
    attackBuffer.allocate(fields * 2 * sizeof(unsigned), GL_DYNAMIC_DRAW);
    clearBuffer(contributionBuffer);
    clearBuffer(attackBuffer);
    clearBuffer(statsBuffer);
    current = 0;
    turn = 0;
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
}

void SlimeSimulation::clearBuffer(ofBufferObject& buffer) {
    buffer.bind(GL_SHADER_STORAGE_BUFFER);
    const unsigned zero = 0;
    glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, &zero);
    buffer.unbind(GL_SHADER_STORAGE_BUFFER);
}

void SlimeSimulation::bindBuffers() {
    agentBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 0);
    trailBuffers[current].bindBase(GL_SHADER_STORAGE_BUFFER, 1);
    trailBuffers[1 - current].bindBase(GL_SHADER_STORAGE_BUFFER, 2);
    contributionBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 3);
    siteBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 4);
    statsBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 5);
    historyBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 7);
    attackBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 6);
}

void SlimeSimulation::step(const Parameters& p) {
    if (p.exploreCost == 0 || p.encroachCost == 0 || p.attackCost == 0 || p.defenseCost == 0 ||
        p.explorationSteps == 0 || p.explorationSteps > 160 ||
        !std::isfinite(p.defenseProbability) || p.defenseProbability < 0 || p.defenseProbability > 1 ||
        p.speed < 0 || p.speed > 2 || p.routeThreshold <= 0 ||
        p.diffusion < 0 || p.diffusion > 1 || p.evaporation < 0 || p.evaporation > 1) {
        throw std::invalid_argument("Invalid simulation parameters");
    }
    // First fund actual attacks, then active defense, then ordinary movement.
    // All decisions sense the same field; cleared segments open next turn.
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    clearBuffer(contributionBuffer);
    clearBuffer(attackBuffer);
    clearBuffer(statsBuffer);
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
    bindBuffers();

    planBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 2);
    movementShader.begin();
    movementShader.setUniform2i("gridSize", width, height);
    movementShader.setUniform1i("turn", turn);
    movementShader.setUniform1i("agentsPerTeam", agentsPerTeam);
    movementShader.setUniform1i("budget", p.budget);
    movementShader.setUniform1i("explorationSteps", p.explorationSteps);
    movementShader.setUniform3i("costs", p.exploreCost, p.encroachCost, p.attackCost);
    movementShader.setUniform1f("speed", p.speed);
    movementShader.setUniform1f("sensorDistance", p.sensorDistance);
    movementShader.setUniform1f("turnAngle", p.turnAngle);
    movementShader.setUniform1f("scentAttraction", p.scentAttraction);
    movementShader.setUniform1f("freshSpace", p.freshSpace);
    movementShader.setUniform1f("trailFollowing", p.trailFollowing);
    movementShader.setUniform1f("avoidance", p.avoidance);
    movementShader.setUniform1f("exploration", p.exploration);
    movementShader.setUniform1f("routeThreshold", p.routeThreshold);
    movementShader.dispatchCompute((teamCount * agentsPerTeam + 127) / 128, 1, 1);
    movementShader.end();
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    combatShader.begin();
    combatShader.setUniform2i("gridSize", width, height);
    combatShader.setUniform1i("turn", turn);
    combatShader.setUniform1i("budget", p.budget);
    combatShader.setUniform1i("defenseCost", p.defenseCost);
    combatShader.setUniform1f("defenseProbability", p.defenseProbability);
    combatShader.setUniform1f("routeThreshold", p.routeThreshold);
    combatShader.dispatchCompute((width * height + 127) / 128, 1, 1);
    combatShader.end();
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    applyShader.begin();
    applyShader.setUniform2i("gridSize", width, height);
    applyShader.setUniform1i("budget", p.budget);
    applyShader.setUniform3i("costs", p.exploreCost, p.encroachCost, p.attackCost);
    applyShader.setUniform1i("turn", turn);
    applyShader.dispatchCompute((teamCount * agentsPerTeam + 127) / 128, 1, 1);
    applyShader.end();
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    trailBuffers[1 - current].bindBase(GL_SHADER_STORAGE_BUFFER, 2);
    trailShader.begin();
    trailShader.setUniform2i("gridSize", width, height);
    trailShader.setUniform1f("diffusion", p.diffusion);
    trailShader.setUniform1f("evaporation", p.evaporation);
    trailShader.dispatchCompute((width * height + 127) / 128, 1, 1);
    trailShader.end();
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
    current = 1 - current;
    ++turn;
}

void SlimeSimulation::draw(float drawWidth, float drawHeight, float routeThreshold, bool showAgents,
                           bool showPheromones, bool showRoutes) {
    trailBuffers[current].bindBase(GL_SHADER_STORAGE_BUFFER, 1);
    displayShader.begin();
    displayShader.setUniform2i("gridSize", width, height);
    displayShader.setUniform2f("drawSize", drawWidth, drawHeight);
    displayShader.setUniform1f("routeThreshold", routeThreshold);
    displayShader.setUniform1i("showPheromones", showPheromones);
    displayShader.setUniform1i("showRoutes", showRoutes);
    displayShader.setUniform4fv("teamColors", &colors[0].x, teamCount);
    ofSetColor(255);
    ofDrawRectangle(0, 0, drawWidth, drawHeight);
    displayShader.end();
    // Show agents independently of their trails, so a waiting colony is visible.
    if (showAgents) {
        agentBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 0);
        agentShader.begin();
        agentShader.setUniform2i("gridSize", width, height);
        agentShader.setUniform2f("drawSize", drawWidth, drawHeight);
        agentShader.setUniform4fv("teamColors", &colors[0].x, teamCount);
        glPointSize(4.0f);
        agentMesh.draw();
        glPointSize(1.0f);
        agentShader.end();
    }
}

std::array<SlimeSimulation::TeamStats, SlimeSimulation::teamCount> SlimeSimulation::readStats() {
    auto data = readBuffer<TeamStats>(statsBuffer, teamCount);
    std::array<TeamStats, teamCount> result;
    std::copy(data.begin(), data.end(), result.begin());
    return result;
}

std::vector<SlimeSimulation::Agent> SlimeSimulation::readAgents() {
    return readBuffer<Agent>(agentBuffer, teamCount * agentsPerTeam);
}

std::vector<glm::vec4> SlimeSimulation::readTrails() {
    return readBuffer<glm::vec4>(trailBuffers[current], static_cast<std::size_t>(width) * height * teamCount);
}

void SlimeSimulation::uploadAgents(const std::vector<Agent>& agents) {
    if (agents.size() != teamCount * agentsPerTeam) throw std::invalid_argument("Agent count must remain fixed");
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    auto prepared = agents;
    bool fresh = std::all_of(prepared.begin(), prepared.end(), [](const Agent& agent) {
        return agent.state.x == 0 && agent.journey.y <= 1 && agent.journey.z == 0;
    });
    if (fresh) {
        for (auto& agent : prepared) {
            agent.journey.x = agent.identity.y;
            agent.journey.y = 1;
        }
    }
    agentBuffer.updateData(prepared);
}

void SlimeSimulation::uploadTrails(const std::vector<glm::vec4>& trails) {
    if (trails.size() != static_cast<std::size_t>(width) * height * teamCount) throw std::invalid_argument("Wrong field size");
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    trailBuffers[current].updateData(trails);
}

std::size_t SlimeSimulation::fieldIndex(int x, int y, unsigned team) const {
    return (static_cast<std::size_t>(y) * width + x) * teamCount + team;
}

void SlimeSimulation::uploadSites(const std::array<glm::vec4, teamCount * sitesPerTeam>& positions) {
    for (unsigned team = 0; team < teamCount; ++team) {
        unsigned count = 0;
        for (unsigned site = 0; site < sitesPerTeam; ++site) count += positions[team * 3 + site].z > 0;
        if (count < 2 || count > 3) throw std::invalid_argument("Each team needs 2 or 3 active sites");
        siteCounts[team] = count;
    }
    sites = positions;
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    siteBuffer.updateData(sizeof(sites), sites.data());
}
