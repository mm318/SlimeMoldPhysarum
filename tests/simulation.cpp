#include "SlimeSimulation.h"
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <queue>
#include <stdexcept>

namespace {
using Simulation = SlimeSimulation;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void checkInitialization(Simulation& simulation) {
    const auto agents = simulation.readAgents();
    const auto& sites = simulation.getSites();
    const auto& counts = simulation.getSiteCounts();
    require(agents.size() == 25 * Simulation::agentsPerTeam, "Fixed population must include all 25 teams");
    unsigned twos = 0, threes = 0;
    for (unsigned team = 0; team < 25; ++team) {
        require(counts[team] == 2 || counts[team] == 3, "Each team needs 2 or 3 sites");
        twos += counts[team] == 2;
        threes += counts[team] == 3;
        unsigned population[3] = {};
        glm::vec2 headings[3] = {};
        for (unsigned local = 0; local < Simulation::agentsPerTeam; ++local) {
            const auto& agent = agents[team * Simulation::agentsPerTeam + local];
            require(agent.identity.x == team, "Agent assigned to wrong team");
            require(agent.identity.y / 3 == team && agent.identity.z / 3 == team, "Targets must belong to own team");
            require(agent.identity.y == agent.identity.z, "Scouts must start without a known destination");
            require(agent.identity.y % 3 < counts[team] && agent.identity.z % 3 < counts[team], "Targeting inactive site");
            require(glm::distance(glm::vec2(agent.position), glm::vec2(sites[agent.identity.y])) <= 1.601f,
                    "Initial agents must be clumped in tight disks around their sites");
            ++population[agent.identity.y % 3];
            headings[agent.identity.y % 3] += glm::vec2(std::cos(agent.position.z), std::sin(agent.position.z));
        }
        for (unsigned site = 0; site < counts[team]; ++site) {
            require(population[site] > 0, "Every site needs agents");
            require(glm::length(headings[site] / float(population[site])) < 0.15f,
                    "Each colony's starting headings must be unbiased");
        }
    }
    require(twos > 0 && threes > 0, "Seeded run should contain both 2-site and 3-site teams");
    simulation.reset(123456);
    const auto repeated = simulation.readAgents();
    require(std::memcmp(agents.data(), repeated.data(), agents.size() * sizeof(agents[0])) == 0,
            "Same seed must reproduce initial conditions");
}

void checkBudgetAndFields(Simulation& simulation) {
    Simulation::Parameters p;
    p.budget = 0;
    p.diffusion = p.evaporation = 0;
    auto before = simulation.readAgents();
    simulation.step(p);
    auto after = simulation.readAgents();
    for (std::size_t i = 0; i < before.size(); ++i) {
        require(before[i].position == after[i].position, "Zero budget must prevent movement");
    }
    for (const auto& stats : simulation.readStats()) require(stats.spent == 0, "Zero budget was exceeded");

    std::vector<glm::vec4> trails(static_cast<std::size_t>(simulation.getWidth()) * simulation.getHeight() * 25, glm::vec4(0));
    simulation.uploadTrails(trails);
    p.budget = 20;
    p.exploreCost = 2;
    simulation.step(p);
    for (const auto& stats : simulation.readStats()) {
        require(stats.spent == 20 && stats.exploring == 10, "Team pool must fund a subset of agents");
        require(stats.encroaching == 0 && stats.attacking == 0, "Empty field should only cost exploration energy");
    }
    bool traffic = false;
    for (auto field : simulation.readTrails()) traffic |= field.w > 0;
    require(traffic, "Ordinary scouting movement must reinforce trails before any site connection");
    for (const auto& agent : simulation.readAgents()) require(agent.state.y == 0, "This trail fixture must not depend on site visits");

    for (std::size_t i = 0; i < trails.size(); i += 25) trails[i].x = 1.0f;
    simulation.uploadTrails(trails);
    simulation.uploadAgents(before);
    p.exploreCost = 1;
    simulation.step(p);
    const auto stats = simulation.readStats();
    require(stats[0].spent == 20 && stats[0].encroaching == 0, "Own scent must not incur enemy cost");
    for (unsigned team = 1; team < 25; ++team) {
        require(stats[team].spent == 18 && stats[team].encroaching == 6,
                "Foreign scent must consume the shared higher-cost budget");
    }

    for (std::size_t i = 0; i < trails.size(); i += 25) trails[i].x = 16;
    simulation.uploadTrails(trails);
    simulation.uploadAgents(before);
    simulation.step(p);
    const auto denseStats = simulation.readStats();
    for (unsigned team = 1; team < 25; ++team) {
        const auto dense = denseStats[team];
        require(dense.attacking == 0 && dense.encroaching == 6,
                "Dense scout pheromone must remain distinct from established enemy routes");
    }

    std::fill(trails.begin(), trails.end(), glm::vec4(0));
    trails[simulation.fieldIndex(0, 0, 24)] = glm::vec4(1, 2, 3, 7);
    simulation.uploadTrails(trails);
    p.budget = 0;
    simulation.step(p);
    const auto preserved = simulation.readTrails();
    require(preserved[simulation.fieldIndex(0, 0, 24)] == glm::vec4(1, 2, 3, 7),
            "Team 25 must preserve all three colony scents and independent route strength");
    for (std::size_t i = 0; i < trails.size(); ++i) {
        require(preserved[i].w == trails[i].w, "Site emissions must not create or alter established routes");
    }
}

void checkCombat(Simulation& simulation) {
    simulation.reset(123456);
    const auto original = simulation.readAgents();
    std::vector<glm::vec4> trails(static_cast<std::size_t>(simulation.getWidth()) * simulation.getHeight() * 25, glm::vec4(0));
    for (std::size_t i = 0; i < trails.size(); i += 25) trails[i].w = 12;
    Simulation::Parameters p;
    p.budget = 24;
    p.diffusion = p.evaporation = 0;
    unsigned attacks = 0, victories = 0;
    for (int turn = 0; turn < 12; ++turn) {
        simulation.uploadAgents(original);
        simulation.uploadTrails(trails);
        simulation.step(p);
        auto stats = simulation.readStats();
        require(stats[0].attacking == 0, "A team cannot attack its own route");
        for (unsigned team = 1; team < 25; ++team) {
            require(stats[team].attacking == 2 && stats[team].spent == 24, "Attacks must use the largest action cost");
            require(stats[team].wins <= stats[team].attacking, "Cannot win more cells than attacked");
            attacks += stats[team].attacking;
            victories += stats[team].wins;
        }
        unsigned erased = 0;
        auto resolved = simulation.readTrails();
        for (std::size_t i = 0; i < resolved.size(); i += 25) {
            require(std::isfinite(resolved[i].w) && resolved[i].w >= 0, "Combat produced invalid pheromone strength");
            erased += resolved[i].w == 0;
        }
        unsigned wins = 0;
        for (auto stat : stats) wins += stat.wins;
        require(erased == wins, "A successful attack must erase exactly its defeated route segment");
        const auto agents = simulation.readAgents();
        for (std::size_t i = Simulation::agentsPerTeam; i < agents.size(); ++i) {
            require(glm::vec2(agents[i].position) == glm::vec2(original[i].position),
                    "Attacker must not walk through a route before combat resolves");
        }
    }
    require(victories > 0 && victories < attacks, "Both attackers and defenders must sometimes win");

    // The strongest team at a cell must also be able to attack a weaker enemy
    // route that overlaps its own, rather than spending energy on itself.
    for (std::size_t i = 0; i < trails.size(); i += 25) trails[i + 1].w = 8;
    simulation.uploadAgents(original);
    simulation.uploadTrails(trails);
    p.attackPower = 1000000;
    p.defense = 0;
    simulation.step(p);
    const auto overlapping = simulation.readStats();
    require(overlapping[0].attacking == 2 && overlapping[0].wins > 0,
            "Overlapping own routes must not prevent attacks on foreign routes");
}

void checkLocality() {
    Simulation first, second;
    first.setup(64, 48, 1234);
    second.setup(64, 48, 5678);
    auto agents = first.readAgents();
    for (auto& agent : agents) {
        agent.position = glm::vec4(2, 2, 0.4f, 0);
        agent.identity.y = agent.identity.z = agent.identity.x * 3;
    }
    first.uploadAgents(agents);
    second.uploadAgents(agents);
    Simulation::Parameters p;
    p.budget = Simulation::agentsPerTeam * p.encroachCost;
    first.step(p);
    second.step(p);
    const auto a = first.readAgents(), b = second.readAgents();
    for (std::size_t i = 0; i < a.size(); ++i) {
        require(a[i].position == b[i].position,
                "Changing distant site locations must not change movement in the same local field");
    }
}

void checkTemporaryRoutes(Simulation& simulation) {
    simulation.reset(123456);
    auto sites = simulation.getSites();
    for (unsigned team = 0; team < 25; ++team) {
        sites[team * 3] = glm::vec4(6, 6, 1, 0);
        sites[team * 3 + 1] = glm::vec4(58, 6, 1, 0);
        sites[team * 3 + 2] = glm::vec4(58, 42, 1, 0);
    }
    simulation.uploadSites(sites);
    auto agents = simulation.readAgents();
    for (auto& agent : agents) agent.position = glm::vec4(30.2f, 24.2f, 0, 0);
    simulation.uploadAgents(agents);
    Simulation::Parameters p;
    p.budget = Simulation::agentsPerTeam * p.encroachCost;
    p.diffusion = p.evaporation = p.exploration = 0;
    p.scentAttraction = p.freshSpace = p.trailFollowing = p.avoidance = 0;
    simulation.step(p);
    auto fields = simulation.readTrails();
    const auto stats = simulation.readStats();
    for (unsigned team = 0; team < 25; ++team) {
        require(stats[team].arrivals == 0, "Intermediate-node fixture must not visit a site");
        require(fields[simulation.fieldIndex(30, 24, team)].w >= p.routeThreshold,
                "Traffic must establish a route at an intermediate node with all sites unvisited");
    }

    // Withdraw traffic: the former route gets no protected lifetime or special
    // ownership status, and dissolves without an enemy having to remove it.
    std::fill(fields.begin(), fields.end(), glm::vec4(0));
    fields[simulation.fieldIndex(30, 24, 0)] = glm::vec4(6, 0, 0, 6);
    simulation.uploadTrails(fields);
    p.budget = 0;
    p.evaporation = 0.2f;
    simulation.step(p);
    auto decaying = simulation.readTrails()[simulation.fieldIndex(30, 24, 0)];
    require(std::abs(decaying.x - decaying.w) < 0.0001f, "Routes must decay at the ordinary scent rate");
    for (int turn = 0; turn < 5; ++turn) simulation.step(p);
    require(simulation.readTrails()[simulation.fieldIndex(30, 24, 0)].w < p.routeThreshold,
            "An abandoned route must disappear without combat or endpoint approval");
}

void checkExploration() {
    Simulation simulation;
    simulation.setup(160, 120, 24680);
    const auto initial = simulation.readAgents();
    Simulation::Parameters p;
    p.budget = Simulation::agentsPerTeam * p.encroachCost;
    p.routeThreshold = 1000;
    p.scentAttraction = p.trailFollowing = p.avoidance = 0;
    for (int i = 0; i < 250; ++i) simulation.step(p);
    const auto agents = simulation.readAgents();
    unsigned dispersed = 0;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        require(std::isfinite(agents[i].position.x) && std::isfinite(agents[i].position.y), "Invalid agent position");
        require(agents[i].position.x >= 0 && agents[i].position.x < simulation.getWidth() &&
                agents[i].position.y >= 0 && agents[i].position.y < simulation.getHeight(), "Agent escaped the world");
        dispersed += glm::distance(glm::vec2(initial[i].position), glm::vec2(agents[i].position)) > 12;
    }
    require(dispersed > agents.size() * 0.6, "Scouts must explore beyond their tight starting colonies");
    for (auto field : simulation.readTrails()) {
        for (int component = 0; component < 4; ++component) {
            require(std::isfinite(field[component]) && field[component] >= 0 &&
                    field[component] <= (component == 3 ? 48 : 16), "Scent and routes must stay finite and bounded");
        }
    }
}

void checkLocalConnections(Simulation& simulation) {
    simulation.reset(123456);
    auto sites = simulation.getSites();
    for (unsigned team = 0; team < 25; ++team) {
        sites[team * 3] = glm::vec4(12, 24, 1, 0);
        sites[team * 3 + 1] = glm::vec4(50, 24, 1, 0);
        sites[team * 3 + 2] = glm::vec4(32, 44, 1, 0);
    }
    simulation.uploadSites(sites);
    auto agents = simulation.readAgents();
    for (std::size_t i = 0; i < agents.size(); ++i) {
        unsigned team = agents[i].identity.x;
        unsigned home = i % 2;
        agents[i].identity.y = agents[i].identity.z = team * 3 + home;
        agents[i].position.x = sites[team * 3 + home].x;
        agents[i].position.y = 24;
    }
    simulation.uploadAgents(agents);
    // A controlled local chemical bridge. Only sensor samples, not target
    // coordinates, can guide the initially random headings along it.
    std::vector<glm::vec4> trails(64 * 48 * 25);
    for (int y = 0; y < 48; ++y) for (int x = 0; x < 64; ++x) {
        for (unsigned team = 0; team < 25; ++team) {
            float t = float(x) / 63;
            trails[simulation.fieldIndex(x, y, team)] = glm::vec4(12 * (1 - t), 12 * t, 0, 0);
        }
    }
    simulation.uploadTrails(trails);
    Simulation::Parameters p;
    p.budget = Simulation::agentsPerTeam * p.encroachCost;
    p.speed = 0.75f;
    p.scentAttraction = 5;
    p.freshSpace = 2;
    p.trailFollowing = p.exploration = p.avoidance = 0;
    p.routeThreshold = 1000;
    for (int i = 0; i < 150; ++i) simulation.step(p);
    bool visited[25][3] = {};
    for (const auto& agent : simulation.readAgents()) {
        if (agent.state.y > 0) visited[agent.identity.x][agent.identity.y % 3] = true;
    }
    const auto fields = simulation.readTrails();
    for (unsigned team = 0; team < 25; ++team) {
        require(visited[team][0] && visited[team][1], "Local scent should guide visits to both connected sites");
        require(!visited[team][2], "The third site must remain unvisited in this partial-network fixture");
        bool reinforced = false;
        for (int x = 20; x < 42; ++x) {
            reinforced |= fields[simulation.fieldIndex(x, 24, team)].w >= Simulation::Parameters().routeThreshold;
        }
        require(reinforced, "A connection between two sites must form a route while the third remains unvisited");
    }
}

void checkRendering(Simulation& simulation) {
    std::vector<glm::vec4> fields(64 * 48 * 25, glm::vec4(0));
    for (std::size_t i = 0; i < fields.size(); i += 25) fields[i] = glm::vec4(12, 0, 0, 12);
    simulation.uploadTrails(fields);
    ofFbo target;
    target.allocate(64, 48, GL_RGBA);
    auto frame = [&](bool scent, bool route, bool agents = false) {
        target.begin();
        ofClear(0, 255);
        ofSetupScreenOrtho(64, 48);
        simulation.draw(64, 48, 6, agents, scent, route);
        target.end();
        ofPixels pixels;
        target.readToPixels(pixels);
        return pixels;
    };
    auto render = [&](bool scent, bool route) { return frame(scent, route).getColor(32, 24); };
    auto background = render(false, false);
    auto scent = render(true, false);
    auto route = render(false, true);
    require(scent.getBrightness() > background.getBrightness(), "Pheromone layer must be visible");
    require(route.getBrightness() > scent.getBrightness() + 50, "Established routes must be distinct from scent");
    for (auto& field : fields) field.w = 0;
    simulation.uploadTrails(fields);
    require(render(false, true) == background, "Scent alone must not render as an established route");
    for (auto& field : fields) field.w = 1;
    simulation.uploadTrails(fields);
    require(render(false, true).getBrightness() > background.getBrightness(), "Developing trails should be visible below the strong-route threshold");

    auto agents = simulation.readAgents();
    for (auto& agent : agents) agent.position = glm::vec4(-100, -100, 0, 0);
    agents[0].position = glm::vec4(32.5f, 24.5f, 0, 0);
    agents[0].state.z = 1;
    simulation.uploadAgents(agents);
    auto exploring = frame(false, false, true);
    agents[0].state.z = 3;
    simulation.uploadAgents(agents);
    auto attacking = frame(false, false, true);
    require(std::memcmp(exploring.getData(), attacking.getData(), exploring.size()) == 0,
            "Attackers and other active agents must use the same appearance");
    bool colored = false;
    for (int y = 0; y < 48; ++y) for (int x = 0; x < 64; ++x) {
        auto color = exploring.getColor(x, y);
        colored |= color.r > color.g + 60 && color.r > color.b + 60;
    }
    require(colored, "Agents must display their team's color rather than white");
}

}

int main() {
#if defined(GLFW_PLATFORM_X11)
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
    try {
        ofGLFWWindowSettings settings;
        settings.setGLVersion(4, 4);
        settings.setSize(64, 64);
        settings.visible = false;
        auto window = ofCreateWindow(settings);
        require(glGetString(GL_VERSION) != nullptr, "OpenGL 4.4 context is required");
        Simulation simulation;
        simulation.setup(64, 48, 123456);
        checkInitialization(simulation);
        std::cout << "PASS initial populations, colony disks, fixed sites, reproducible reset\n";
        checkBudgetAndFields(simulation);
        std::cout << "PASS pooled budgets, zero energy, terrain costs, 25 independent fields\n";
        checkCombat(simulation);
        std::cout << "PASS contested combat, attacker losses, route erasure\n";
        checkTemporaryRoutes(simulation);
        std::cout << "PASS intermediate-node routes, ordinary decay, voluntary abandonment\n";
        checkLocality();
        std::cout << "PASS no steering information from distant site coordinates\n";
        checkExploration();
        std::cout << "PASS exploration beyond starting disks, bounded independent fields\n";
        checkLocalConnections(simulation);
        std::cout << "PASS local chemical bridges guide site discovery and route reinforcement\n";
        checkRendering(simulation);
        std::cout << "PASS pheromones and established routes render as independent layers\n";
        require(glGetError() == GL_NO_ERROR, "OpenGL error during simulation checks");
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
