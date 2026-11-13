// Included after the Agent, Sites, History and Statistics buffer declarations.
void finishReturn(inout Agent agent, uint index) {
    agent.state.x = 0.0;
    agent.state.w += 1.0;
    agent.journey.y = 1u;
    agent.journey.z = 0u;
    agent.identity.w = hash(agent.identity.w + uint(agent.state.w) * 7919u + 17u);
    agent.position.z = random01(agent.identity.w) * 6.28318530718;
    history[index * HISTORY_CAPACITY] = sites[agent.journey.x].xy;
    atomicAdd(stats[agent.identity.x].homeReturns, 1u);
}

bool reserveEnergy(uint team, uint cost) {
    uint used = atomicAdd(stats[team].spent, 0u);
    while (used + cost <= uint(budget)) {
        uint previous = atomicCompSwap(stats[team].spent, used, used + cost);
        if (previous == used) return true;
        used = previous;
    }
    return false;
}
